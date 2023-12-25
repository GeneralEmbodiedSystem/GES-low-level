"""
Define the grasp skill.
"""
from servers.utils import PipeManager

import numpy as np

from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.utils.control_utils import IKSolver
from omnigibson.utils.hand_skill_utils import check_joint_move


left_or_right = 'left'


def emit_grasp(pipe: PipeManager, global_states: dict):
    """Control gripper to grasp. (simple version of grasping task)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
    """
    
    robot: TiagoAdvanced = global_states['robot']
    
    global_states['active_actions']['hand_action'] = {
        'func': 'grasp',
        'args': {},
    }
    
    # hand action buffer
    buffer: dict = global_states['active_actions']['hand_action']

    if 'tgt_joint_pos' not in buffer:
        buffer['tgt_joint_pos'] = {}
    buffer['tgt_joint_pos']['grasp'] = ([0., 0.], robot.gripper_control_idx[left_or_right])


def execute_grasp(pipe: PipeManager, global_states: dict):
    """generate action for grasp

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
    
    # check end of a grasp according to the joint pos delta
    end_ik_flag = check_joint_move(robot, buffer, robot.gripper_control_idx[left_or_right])
    
    # last ik do not move && last ik ends
    if end_ik_flag:
        pipe.write(
            data="grasp success",
            role="server"
        )
        global_states['active_actions']['hand_action'] = None
        
        # hand action is over
        return action
    
    # return None means using hand_custom_step(robot, global_states['active_actions']['hand_action']) 
    # instead of env.step(action).
    return None
