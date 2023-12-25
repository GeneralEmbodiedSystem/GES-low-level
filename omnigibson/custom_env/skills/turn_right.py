"""
Define the turn_right skill.
"""
from servers.utils import PipeManager

from omnigibson.custom_env.utils.constant_utils import MEASURED_TURN_SPEED


def emit_turn_right(pipe: PipeManager, global_states: dict, angle: int):
    """Turn the robot right. This function only set a `turn_right` state. 
    the actual movement (generate action at each time) is done in execute_turn_right

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
        angle (int): angle to turn right (`angle` in [1, 180]).
    """
    global_states['active_actions']['move_action'] = {
        'func': 'turn_right',
        'args': {
            'angle': angle,
        }
    }
    # Reset the steps that have been turned
    global_states['turn_steps'] = 0
    
    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_turn_right(pipe: PipeManager, global_states: dict, angle: int):
    """generate action for turn_right (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
        angle (int): angle to turn right (`angle` in [1, 180]).
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    action = global_states['action']
    if global_states['turn_steps'] < angle:
        global_states['turn_steps'] += 1
        action[2] = -MEASURED_TURN_SPEED
    else:
        pipe.write(
            data="turn_right success",
            role="server"
        )
        # Erase the corresponding active action
        global_states['active_actions']['move_action'] = None

    return action
