"""
Define the open gripper skill.
"""
from servers.utils import PipeManager

import numpy as np

from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.utils.control_utils import IKSolver
from omnigibson.custom_env.utils.eular_utils import check_joint_still


left_or_right = 'left'


def emit_open_gripper(pipe: PipeManager, global_states: dict):
    """Control gripper to open gripper.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
    """
    global_states['active_actions']['hand_action'] = {
        'func': 'open_gripper',
        'args': {},
    }
    
    # robot states buffer
    if 'robot_states' not in global_states['active_actions']:
        global_states['active_actions']['robot_states'] = {}
    buffer: dict = global_states['active_actions']['robot_states']
    
    # set gripper state
    buffer['gripper_state'] = 'open'


def execute_open_gripper(pipe: PipeManager, global_states: dict):
    """generate action for opening the gripper

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    action = global_states['action']
    robot: TiagoAdvanced = global_states['robot']
    
    # hand action buffer
    buffer: dict = global_states['active_actions']['hand_action']
    
    # check end of a opening gripper action according to the joint pos delta
    end_ik_flag = check_joint_still(robot, buffer, robot.gripper_control_idx[left_or_right])
    
    # last ik do not move && last ik ends
    if end_ik_flag:
        pipe.write(
            data="open_gripper success",
            role="server"
        )
        global_states['active_actions']['hand_action'] = None
        
        print(f'mk open_gripper success')
        
        # hand action is over
        return action
    
    # return None means using hand_custom_step(robot, global_states['active_actions']['hand_action']) 
    # instead of env.step(action).
    return None
