"""
Define the reach skill.
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

from omnigibson.utils.hand_skill_utils import check_joint_move

left_or_right = 'left'


def emit_reach(pipe: PipeManager, global_states: dict, object: str):
    """Control arm to reach a object.

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
    
    # waypoints: [((x1, y1, z1), eps1), ((x2, y2, z2), eps2)]
    # eps means the max distance error to the waypoint we can tolerate
    waypoints = []
    waypoints_scratch = {
        'waypoints': waypoints,
        'current_waypoint_index': 0,
    }
    
    # plan rule-based waypoints
    object_pos = get_pos_by_prim_path_physxApi(f'/World/{object}/base_link')
    if object in ['trash_can']:
        waypoints.append((object_pos + np.array([0.0, 0.0, 0.3]), 1e-4))
    else:
        waypoints.append((object_pos + np.array([0.0, 0.0, 0.15]), 1e-5))
        waypoints.append((object_pos, 1e-1))
        # waypoints.append((object, 5e-2))
    
    # define single_reach_scratch init state generation function
    def single_reach_scratch_init_func():
        return {}
    
    # define the object name to reach in local vars
    global_states['active_actions']['hand_action'] = {
        'func': 'reach',
        'args': {
            'object': object,
        },
        'ik_solver': ik_solver,
        
        'waypoints_scratch': waypoints_scratch,
        
        'single_reach_scratch': None, # no use currently
        'single_reach_scratch_init_func': single_reach_scratch_init_func, # no use currently
    }
    
    # hand action buffer
    buffer: dict = global_states['active_actions']['hand_action']

    # start navigate to waypoint
    start_navigate_to_waypoint(buffer, robot)
    print('operator: start navigate to next waypoint')


def execute_reach(pipe: PipeManager, global_states: dict, object: str):
    """generate action for reach

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
    single_reach_scratch: dict = buffer['single_reach_scratch']
    
    # extract params from buffer
    ik_solver: IKSolver = buffer['ik_solver']
    waypoints_scratch = buffer['waypoints_scratch']
    waypoints = waypoints_scratch['waypoints']
    current_waypoint_index = waypoints_scratch['current_waypoint_index']
    
    # check end of a ik according to the joint pos delta
    eef_name = robot.eef_link_names[robot.default_arm]
    this_tgt_waypoint = parse_waypoint_pos(waypoints[current_waypoint_index][0])
    eef_current_pos = get_pos_by_prim_path_physxApi(f'{robot.prim_path}/{eef_name}')
    to_move_distance = calculate_distance(this_tgt_waypoint, eef_current_pos)
    # eps = 1e-5 # can not end, but can successfully reach
    eps = waypoints[current_waypoint_index][1]
    reach_waypoint_flag = (to_move_distance < eps)
    joint_move_flag = check_joint_move(robot, single_reach_scratch, robot.gripper_control_idx[left_or_right])
    
    print(f'operator: to_move_distance: {to_move_distance}')
    
    # this waypoint navigation ends
    if reach_waypoint_flag:
        end_navigation_of_a_waypoint(buffer, robot)
        print('operator: reach waypoint success!')
    
        if check_waypoints_end(buffer):
            pipe.write(
                data="reach success",
                role="server"
            )
            global_states['active_actions']['hand_action'] = None
            
            print(f'operator: reach {object} success')
            
            # hand action is over
            return action
        else:
            start_navigate_to_waypoint(buffer, robot)
            print('operator: start navigate to next waypoint')
    elif not joint_move_flag:
        execute_ik(ik_solver, robot, np.array(parse_waypoint_pos(waypoints[current_waypoint_index][0])), buffer)
    
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
    
    # init single_reach_scratch
    buffer['single_reach_scratch'] = buffer['single_reach_scratch_init_func']()
    
    # ik
    execute_ik(buffer['ik_solver'], robot, np.array(parse_waypoint_pos(waypoints[current_waypoint_index][0])), buffer)
    
    # keep releasing when reach
    buffer['tgt_joint_pos']['release'] = ([0.045, 0.045], robot.gripper_control_idx[left_or_right])

def end_navigation_of_a_waypoint(buffer: dict, robot: TiagoAdvanced):
    # extra params
    waypoints_scratch = buffer['waypoints_scratch']
    
    # increase current_waypoint_index
    waypoints_scratch['current_waypoint_index'] += 1
    
    # empty single_reach_scratch
    buffer['single_reach_scratch'] = None
    
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
        tolerance_pos = 0.00001
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
