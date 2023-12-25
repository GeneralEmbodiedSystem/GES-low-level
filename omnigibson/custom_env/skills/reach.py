"""
Define the reach skill.
"""
from servers.utils import PipeManager

import numpy as np

from omni.physx import get_physx_interface
from pxr import Gf
from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.utils.control_utils import IKSolver
from omnigibson.utils.hand_skill_utils import check_joint_move

left_or_right = 'left'

def emit_reach(pipe: PipeManager, global_states: dict, object: str):
    """Control arm to reach a object.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
        object (str): the name in prim path defined in isaac, e.g. 
            'delicious_apple' in '/World/delicious_apple'
    """
    
    robot: TiagoAdvanced = global_states['robot']
    
    # define ik solver
    ik_solver = IKSolver(
        robot_description_path=robot.robot_arm_descriptor_yamls[robot.default_arm],
        robot_urdf_path=robot.urdf_path,
        default_joint_pos=robot.get_joint_positions()[robot.ik_control_idx],
        eef_name=robot.eef_link_names[robot.default_arm],
    )
    
    print('''I'm in reach operator!!''')
    
    # rule-based procedures
    
    # define the object name to reach in local vars
    global_states['active_actions']['hand_action'] = {
        'func': 'reach',
        'args': {
            'object': object,
        },
        # 'procedures': 
        'last_joint_pos': None,
        'ik_has_effect': True,
        'ik_solver': ik_solver,
    }
    
    # hand action buffer
    buffer: dict = global_states['active_actions']['hand_action']

    execute_ik(ik_solver, robot, get_pos_by_prim_path_physxApi(f'/World/{object}/base_link'), buffer)


def execute_reach(pipe: PipeManager, global_states: dict, object: str):
    """generate action for reach

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
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
    
    # extract params from buffer
    ik_solver: IKSolver = buffer['ik_solver']
    
    # check end of a ik according to the joint pos delta
    end_ik_flag = check_joint_move(robot, buffer, robot.gripper_control_idx[left_or_right])
    
    # update ik_has_effect, ik_has_effect==True if the robot reach at least once 
    # in this ik.
    ik_has_effect = buffer.get('ik_has_effect', False)
    ik_has_effect |= (not end_ik_flag)
    buffer['ik_has_effect'] = ik_has_effect
    

    if not ik_has_effect and end_ik_flag:
        pipe.write(
            data="reach success",
            role="server"
        )
        global_states['active_actions']['hand_action'] = None
        
        # hand action is over
        return action
    
    if end_ik_flag:
        execute_ik(ik_solver, robot, get_pos_by_prim_path_physxApi(f'/World/{object}/base_link'), buffer)
    
    # return None means using hand_custom_step(robot, global_states['active_actions']['hand_action']) 
    # instead of env.step(action).
    return None


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
    @param prim_path: e.g. /world/robot/eyes/base_link
    @return position/translate: np.ndarray([x, y, z])
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
