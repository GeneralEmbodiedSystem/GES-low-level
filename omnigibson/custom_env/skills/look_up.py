"""
Define the grasp skill.
"""
from servers.utils import PipeManager

import numpy as np

from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.utils.hand_skill_utils import check_joint_move



def emit_look_up(pipe: PipeManager, local_vars: dict, degree: float):
    """Control the camera to look up.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        degree: the degree of the camera to look up.
    """
    
    robot: TiagoAdvanced = local_vars['robot']
    
    local_vars['active_actions']['camera_action'] = {
        'func': 'look_up',
        'args': {},
    }
    
    # hand action buffer
    buffer: dict = local_vars['active_actions']['camera_action']

    buffer['tgt_joint_pos']['camera'] = ([0., 0.], robot.camera_control_idx)


def execute_grasp(pipe: PipeManager, local_vars: dict):
    """generate action for grasp

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    action = local_vars['action']
    robot: TiagoAdvanced = local_vars['robot']
    
    # hand action buffer
    buffer: dict = local_vars['active_actions']['hand_action']
    
    # check end of a grasp according to the joint pos delta
    end_ik_flag = check_joint_move(robot, buffer, robot.gripper_control_idx[left_or_right])
    
    # last ik do not move && last ik ends
    if end_ik_flag:
        pipe.write(
            data="grasp success",
            role="server"
        )
        local_vars['active_actions']['hand_action'] = None
        
        # hand action is over
        return action
    
    # return None means using hand_custom_step(robot, local_vars['active_actions']['hand_action']) 
    # instead of env.step(action).
    return None
