"""
Define the look down skill.
"""
from servers.utils import PipeManager

import numpy as np

from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.utils.hand_skill_utils import check_joint_move
from omnigibson.utils.camera_skill_utils import execute_look_down


ACTION_CATAGORY = 'camera_action'
TASK_NAME = 'look_down'

def emit_look_down(pipe: PipeManager, local_vars: dict, degree: float):
    """Control the camera to look down.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        degree: the degree of the camera to look down.
    """
    
    robot: TiagoAdvanced = local_vars['robot']
    
    local_vars['active_actions'][ACTION_CATAGORY] = {
        'func': TASK_NAME,
        'args': {
            'degree': degree,
        },
    }
    
    # hand action buffer
    buffer: dict = local_vars['active_actions'][ACTION_CATAGORY]

    execute_look_down(robot, buffer, degree)

def execute_look_down(pipe: PipeManager, local_vars: dict, degree: float):
    """generate action for look down

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        degree: the degree of the camera to look down.
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    action = local_vars['action']
    robot: TiagoAdvanced = local_vars['robot']
    
    # hand action buffer
    buffer: dict = local_vars['active_actions'][ACTION_CATAGORY]
    
    # check end of a look down according to the joint pos delta
    end_ik_flag = check_joint_move(robot, buffer, robot.camera_control_idx)
    
    # last ik do not move && last ik ends
    if end_ik_flag:
        pipe.write(
            data=f"{TASK_NAME} success",
            role="server"
        )
        local_vars['active_actions'][ACTION_CATAGORY] = None
        
        # hand action is over
        return action
    
    # return None means using hand_custom_step(robot, local_vars['active_actions'][ACTION_CATAGORY]) 
    # instead of env.step(action).
    return None
