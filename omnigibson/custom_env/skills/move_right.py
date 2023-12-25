"""
Define the move_right skill.
"""
from servers.utils import PipeManager


def emit_move_right(pipe: PipeManager, global_states: dict, vel_r: float):
    """Move the robot right. This function only set a `move_right` state. 
    the actual movement (generate action at each time) is done in execute_move_right

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        vel_r (float): velocity to move right (`vel_r >= 0`).
    """
    global_states['active_actions']['move_action'] = {
        'func': 'move_right',
        'args': {
            'vy': -vel_r
        }
    }
    # The velocity has been set, so we can send the ack
    pipe.write(
        data="move_right success",
        role="server"
    )


def execute_move_right(pipe: PipeManager, global_states: dict, vy: float):
    """generate action for move_right (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        vy: velocity to move along y axis. It is negative when moving right.
    """
    action = global_states['action']
    action[1] = vy

    return action
