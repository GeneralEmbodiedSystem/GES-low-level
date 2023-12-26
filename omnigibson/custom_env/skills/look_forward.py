"""
Define the look forward skill.
"""
from servers.utils import PipeManager

import numpy as np

from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.custom_env.utils.eular_utils import check_joint_still
from omnigibson.utils.camera_skill_utils import execute_look_forward


ACTION_CATAGORY = 'camera_action'
TASK_NAME = 'look_forward'

def emit_look_forward(pipe: PipeManager, global_states: dict):
    """Control the camera to look forward.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
    """
    
    robot: TiagoAdvanced = global_states['robot']
    
    global_states['active_actions'][ACTION_CATAGORY] = {
        'func': TASK_NAME,
        'args': {},
    }
    
    # hand action buffer
    buffer: dict = global_states['active_actions'][ACTION_CATAGORY]

    execute_look_forward(robot, buffer)

def execute_look_forward(pipe: PipeManager, global_states: dict):
    """generate action for look forward

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
    buffer: dict = global_states['active_actions'][ACTION_CATAGORY]
    
    # check end of a look forward according to the joint pos delta
    end_ik_flag = check_joint_still(robot, buffer, robot.camera_control_idx)
    
    # last ik do not move && last ik ends
    if end_ik_flag:
        pipe.write(
            data=f"{TASK_NAME} success",
            role="server"
        )
        global_states['active_actions'][ACTION_CATAGORY] = None
        
        # hand action is over
        return action
    
    # return None means using hand_custom_step(robot, global_states['active_actions'][ACTION_CATAGORY]) 
    # instead of env.step(action).
    return None
