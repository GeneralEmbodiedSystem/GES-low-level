"""
Define the raise_gripper skill.
"""
from servers.utils import PipeManager

import numpy as np

from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.utils.control_utils import IKSolver
from omnigibson.custom_env.utils.eular_utils import check_joint_still


left_or_right = 'left'


def emit_raise_gripper(pipe: PipeManager, global_states: dict):
    """Control gripper to raise.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
    """
    # extract the robot
    robot: TiagoAdvanced = global_states['robot']
    
    # define the object name to move the gripper to in local vars
    global_states['active_actions']['hand_action'] = {
        'func': 'raise_gripper',
        'args': {},
        
        # store the target joint pos
        'tgt_joint_pos': {},
    }
    
    # hand buffer
    buffer: dict = global_states['active_actions']['hand_action']
    
    # set target position
    arm_control_idx = robot.arm_control_idx[left_or_right]
    arm_default_joint_pos = robot.default_joint_pos[arm_control_idx]
    buffer['tgt_joint_pos']['raise_gripper'] = (arm_default_joint_pos, arm_control_idx)


def execute_raise_gripper(pipe: PipeManager, global_states: dict):
    """generate action for raising gripper

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
    
    # check end of a raising gripper action according to the joint pos delta
    end_ik_flag = check_joint_still(
        robot, buffer, robot.arm_control_idx[left_or_right], 1e-4
    )
    
    # last ik do not move && last ik ends
    if end_ik_flag:
        pipe.write(
            data="raise_gripper success",
            role="server",
        )
        global_states['active_actions']['hand_action'] = None
        
        print(f'mk raise_gripper success')
        
        # hand action is over
        return action
    
    # return None means using hand_custom_step(robot, global_states['active_actions']['hand_action']) 
    # instead of env.step(action).
    return None
