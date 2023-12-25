"""
Define the turn_left skill.
"""
import numpy as np

from servers.utils import PipeManager

from omnigibson.utils.transform_utils import quat2euler

from omnigibson.robots import TiagoAdvanced
from omnigibson.custom_env.utils.constant_utils import TURN_SPEED
from omnigibson.custom_env.utils.eular_utils import check_turn_end


def emit_turn_left(pipe: PipeManager, global_states: dict, angle: int):
    """Turn the robot left. This function only set a `turn_left` state. 
    the actual movement (generate action at each time) is done in execute_turn_left

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        angle (int): angle to turn left (`angle` in [1, 180]).
    """
    # extract the robot
    robot: TiagoAdvanced = global_states['robot']
    
    global_states['active_actions']['move_action'] = {
        'func': 'turn_left',
        'args': {
            'angle': angle,
        },
        'init_z_angle': quat2euler(robot.get_orientation())[2],
        'left_or_right': 'left',
    }
    
    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_turn_left(pipe: PipeManager, global_states: dict, angle: int):
    """generate action for turn_left (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        angle (int): angle to turn left (`angle` in [1, 180]).
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    
    # extract params
    action = global_states['action']
    robot: TiagoAdvanced = global_states['robot']
    init_z_angle = global_states['active_actions']['move_action']['init_z_angle']
    left_or_right = global_states['active_actions']['move_action']['left_or_right']

    # convert angle to radian
    radian = angle / 180 * np.pi

    if not check_turn_end(init_z_angle, left_or_right, radian, quat2euler(robot.get_orientation())[2]):
        action[2] = TURN_SPEED
    else:
        pipe.write(
            data="turn_left success",
            role="server"
        )
        # Erase the corresponding active action
        global_states['active_actions']['move_action'] = None
    return action
