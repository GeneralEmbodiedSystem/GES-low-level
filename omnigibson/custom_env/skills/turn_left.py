"""
Define the turn_left skill.
"""
from servers.utils import PipeManager


def emit_turn_left(pipe: PipeManager, global_states: dict, angle: int):
    """Turn the robot left. This function only set a `turn_left` state. 
    the actual movement (generate action at each time) is done in execute_turn_left

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
        angle (int): angle to turn left (`angle` in [1, 180]).
    """
    global_states['active_actions']['move_action'] = {
        'func': 'turn_left',
        'args': {
            'angle': angle,
        }
    }
    # Reset the steps that have been turned
    global_states['turn_steps'] = 0
    
    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_turn_left(pipe: PipeManager, global_states: dict, angle: int):
    """generate action for turn_left (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
        angle (int): angle to turn left (`angle` in [1, 180]).
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    action = global_states['action']

    if global_states['turn_steps'] < angle:
        global_states['turn_steps'] += 1
        action[2] = 0.341
    else:
        pipe.write(
            data="turn_left success",
            role="server"
        )
        # Erase the corresponding active action
        global_states['active_actions']['move_action'] = None

    return action
