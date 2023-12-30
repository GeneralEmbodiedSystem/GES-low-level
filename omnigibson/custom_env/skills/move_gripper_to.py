"""
Define the moving gripper to skill.
"""
from typing import Union
import numpy as np

from servers.utils import PipeManager

from omni.physx import get_physx_interface
from pxr import Gf
from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.utils.control_utils import IKSolver

from omnigibson.custom_env.utils.physx_utils import get_pos_by_prim_path_physxApi
from omnigibson.custom_env.utils.robot_utils import get_ik_solver
from omnigibson.custom_env.utils.eular_utils import calculate_distance

from omnigibson.custom_env.utils.eular_utils import check_joint_still

left_or_right = 'left'


def emit_move_gripper_to(pipe: PipeManager, global_states: dict, object: str):
    """Control arm to move gripper to a object

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        object (str): the name in prim path defined in isaac, e.g. 
            'delicious_apple' in '/World/delicious_apple'
    """
    
    # extract the robot
    robot: TiagoAdvanced = global_states['robot']
    
    # define ik solver
    ik_solver = get_ik_solver(robot)
    
    # waypoints: [(x1, y1, z1), (x2, y2, z2)]
    # eps means the max distance error to the waypoint we can tolerate
    waypoints = []
    waypoints_scratch = {
        'pre_fetch_flag': False,
        'waypoints': waypoints,
        'current_waypoint_index': 0,
    }
    
    # if object in ['trash_can']:
    #     waypoints_scratch['pre_fetch_flag'] = True
    
    # pre fetch joint position
    pre_fetch_joint_position = np.array([0.85500056, 0.06865133, 0.84170145, 
                                         2.174793, -0.8766601, -0.9809551, -2.0943947])
    
    # plan rule-based waypoints
    object_pos = get_pos_by_prim_path_physxApi(f'/World/{object}/base_link')
    if object in ['trash_can']:
        waypoints.append(object_pos + np.array([0.0, 0.0, 0.4]))
    else:
        waypoints.append(object_pos + np.array([0.0, 0.0, 0.1]))
        # waypoints.append((object_pos, 3e-2))
        # waypoints.append((object_pos, 5e-2))
        waypoints.append(object_pos + np.array([-0.02, 0.02, -0.05]))
        # waypoints.append((object, 3e-2))
    
    # define single_navigation_scratch init state generation function
    def single_navigation_scratch_init_func():
        return {
            'still_times': 0
        }
    
    # define the object name to move the gripper to in local vars
    global_states['active_actions']['hand_action'] = {
        'func': 'move_gripper_to',
        'args': {
            'object': object,
        },
        
        # store the target joint pos
        'tgt_joint_pos': {},
        
        'ik_quat': None,
        
        'ik_solver': ik_solver,
        
        'waypoints_scratch': waypoints_scratch,

        # The number of times running ik
        'ik_times': 0,
        
        'single_navigation_scratch': None, # no use currently
        'single_navigation_scratch_init_func': single_navigation_scratch_init_func, # no use currently
    }
    
    # hand action buffer
    buffer: dict = global_states['active_actions']['hand_action']
    
    # specify ik quat
    if object in ['trash_can']:
        buffer['ik_quat'] = None
    else:
        buffer['ik_quat'] = np.array([0, 0, 0, 1])

    # pre fetch
    buffer['tgt_joint_pos'] = {}
    buffer['tgt_joint_pos']['ik'] = (pre_fetch_joint_position, robot.arm_control_idx[left_or_right])

    # ugly implementation
    if waypoints_scratch['pre_fetch_flag']:
        start_navigate_to_waypoint(buffer, robot)
        print('operator: start navigate to next waypoint')

    # start navigate to waypoint
    # start_navigate_to_waypoint(buffer, robot)
    # print('operator: start navigate to next waypoint')


def execute_move_gripper_to(pipe: PipeManager, global_states: dict, object: str):
    """generate action for move gripper to a object (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        object (str): the name in prim path defined in isaac, e.g. 
            'delicious_apple' in '/World/delicious_apple'
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    action = global_states['action']
    robot: TiagoAdvanced = global_states['robot']
    
    # hand action buffer
    buffer: dict = global_states['active_actions']['hand_action']
    single_navigation_scratch: dict = buffer['single_navigation_scratch']
    
    # extract params from buffer
    ik_solver: IKSolver = buffer['ik_solver']
    waypoints_scratch = buffer['waypoints_scratch']
    waypoints = waypoints_scratch['waypoints']
    current_waypoint_index = waypoints_scratch['current_waypoint_index']
    
    # pre fetch
    if not waypoints_scratch['pre_fetch_flag']:
        if calculate_distance(robot.get_joint_positions()[buffer['tgt_joint_pos']['ik'][1]], 
                              buffer['tgt_joint_pos']['ik'][0]) < 1e-3:
            waypoints_scratch['pre_fetch_flag'] = True
            start_navigate_to_waypoint(buffer, robot)
            print('operator: start navigate to next waypoint')
        return None
    
    # check end of a ik according to the joint pos delta
    eef_name = robot.eef_link_names[robot.default_arm]
    this_tgt_waypoint = parse_waypoint_pos(waypoints[current_waypoint_index])
    eef_current_pos = get_pos_by_prim_path_physxApi(f'{robot.prim_path}/{eef_name}')
    to_move_distance = calculate_distance(this_tgt_waypoint, eef_current_pos)
    # Wether the robot has reached the target waypoint
    EPSILON = 1e-5
    navigation_success_flag = (to_move_distance < EPSILON)
    joint_still_flag = check_joint_still(
        robot, single_navigation_scratch, robot.arm_control_idx[left_or_right], 1e-4
    )
    
    # update still_times
    if joint_still_flag:
        single_navigation_scratch['still_times'] += 1
    else:
        single_navigation_scratch['still_times'] = 0
    
    # print(f'operator: to_move_distance: {to_move_distance}')
    # print(f'operator: robot.joint_velocity_limits: {robot.max_joint_velocities[robot.arm_control_idx[left_or_right]]}')
    # print(f'operator: robot jointpos: {robot.get_joint_positions()[robot.arm_control_idx[left_or_right]]}')
    # print(f'operator: waypoint: {waypoints[current_waypoint_index]}')
    
    # this waypoint navigation ends
    MAX_IK_TIMES = 10
    if navigation_success_flag or buffer['ik_times'] > MAX_IK_TIMES:
        end_navigation_of_a_waypoint(buffer, robot)
        print('operator: navigate to target waypoint success!')
    
        if check_waypoints_end(buffer):
            pipe.write(
                data="move_gripper_to success",
                role="server"
            )
            global_states['active_actions']['hand_action'] = None
            
            print(f'operator: move_gripper_to {object} success')
            
            # hand action is over
            return action
        else:
            start_navigate_to_waypoint(buffer, robot)
            print('operator: start navigate to next waypoint')
    elif single_navigation_scratch['still_times'] > 5:
        # pass
        print('operator: run ik!')
        execute_ik(
            ik_solver, robot, np.array(parse_waypoint_pos(waypoints[current_waypoint_index])), buffer,
            quat=buffer['ik_quat'],
        )
        single_navigation_scratch['still_times'] = 0
        buffer['ik_times'] += 1
    
    # return None means using hand_custom_step(robot, global_states['active_actions']['hand_action']) 
    # instead of env.step(action).
    return None

def parse_waypoint_pos(waypoint: Union[str, np.ndarray]):
    if isinstance(waypoint, str):
        return get_pos_by_prim_path_physxApi(f'/World/{waypoint}/base_link')
    return waypoint

def start_navigate_to_waypoint(buffer: dict, robot: TiagoAdvanced):
    # extra params
    waypoints_scratch = buffer['waypoints_scratch']
    waypoints = waypoints_scratch['waypoints']
    current_waypoint_index = waypoints_scratch['current_waypoint_index']
    
    # init single_navigation_scratch
    buffer['single_navigation_scratch'] = buffer['single_navigation_scratch_init_func']()
    
    # ik
    execute_ik(
        buffer['ik_solver'], robot, np.array(parse_waypoint_pos(waypoints[current_waypoint_index])), buffer,
        quat=buffer['ik_quat'],
    )
    
    # keep releasing when move gripper to
    buffer['tgt_joint_pos']['release'] = ([0.045, 0.045], robot.gripper_control_idx[left_or_right])

def end_navigation_of_a_waypoint(buffer: dict, robot: TiagoAdvanced):
    # extra params
    waypoints_scratch = buffer['waypoints_scratch']
    
    # increase current_waypoint_index
    waypoints_scratch['current_waypoint_index'] += 1
    
    # empty single_navigation_scratch
    buffer['single_navigation_scratch'] = None
    
    # empty target joint pos
    if 'tgt_joint_pos' in buffer:
        for to_pop in ['release', 'ik']:
            if to_pop in buffer['tgt_joint_pos']:
                buffer['tgt_joint_pos'].pop(to_pop)

def check_waypoints_end(buffer: dict):
    # extra params
    waypoints_scratch = buffer['waypoints_scratch']
    waypoints = waypoints_scratch['waypoints']
    current_waypoint_index = waypoints_scratch['current_waypoint_index']
    
    if current_waypoint_index >= len(waypoints):
        return True
    return False

# Define a helper function for executing specific end-effector commands using the ik solver
def execute_ik(ik_solver, robot: TiagoAdvanced, pos, buffer: dict, quat=None, max_iter=100):
    control_idx = robot.ik_control_idx
    joint_pos = ik_solver.solve(
        target_pos=pos - robot.get_position(), # local target quaternion orientation in robot's base coordinate frame
        target_quat=quat,
        max_iterations=max_iter,
        initial_joint_pos=robot.get_joint_positions()[control_idx],
        # tolerance_pos = 0.00001
    )

    # set tgt position
    # robot.set_joint_positions(joint_pos, indices=control_idx, drive=True)
    if 'tgt_joint_pos' not in buffer: 
        buffer['tgt_joint_pos'] = {}
    buffer['tgt_joint_pos']['ik'] = (joint_pos, control_idx)


def get_pos_by_prim_path_physxApi(prim_path: str):
    '''
    get position of the prim_path in physxApi
    
    Args:
        prim_path: prim_path of the object
            e.g. /world/robot/eyes/base_link
    '''
    
    curr_transform = get_physx_interface().get_rigidbody_transformation(prim_path)
    rv_success = curr_transform["ret_val"]
    if rv_success:
        curr_pos_f3 = curr_transform["position"]
        curr_pos = Gf.Vec3d(curr_pos_f3[0], curr_pos_f3[1], curr_pos_f3[2])
        curr_rot_f4 = curr_transform["rotation"]
        curr_rot_quat = Gf.Quatd(curr_rot_f4[3], curr_rot_f4[0], curr_rot_f4[1], curr_rot_f4[2])
        return np.array(curr_pos)
    return None
