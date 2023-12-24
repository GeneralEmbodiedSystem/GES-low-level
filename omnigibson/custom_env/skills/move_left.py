"""
Define the move_left skill.
"""
from servers.utils import PipeManager


def emit_move_left(pipe: PipeManager, local_vars: dict, vel_l: float):
    """Move the robot left. This function only set a `move_left` state. 
    the actual movement (generate action at each time) is done in execute_move_left

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vel_l (float): velocity to move left (`vel_l >= 0`).
    """
    local_vars['active_actions']['move_action'] = {
        'func': 'move_left',
        'args': {
            'vy': vel_l
        }
    }
    # The velocity has been set, so we can send the ack
    pipe.write(
        data="move_left success",
        role="server"
    )


def execute_move_left(pipe: PipeManager, local_vars: dict, vy: float):
    """generate action for move_left (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vy: velocity to move along y axis. It is positive when moving left.
    """
    action = local_vars['action']
    action[1] = vy

    return action
