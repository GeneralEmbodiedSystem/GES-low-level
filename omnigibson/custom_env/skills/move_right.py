"""
Define the move_right skill.
"""
from servers.utils import PipeManager


def run_move_right(pipe: PipeManager, local_vars: dict, vel_r: float):
    """Move the robot right. This function only set a `move_right` state. 
    the actual movement (generate action at each time) is done in execute_move_right

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vel_r (float): velocity to move right (`vel_r >= 0`).
    """
    local_vars['active_actions']['move_action'] = {
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


def execute_move_right(pipe: PipeManager, local_vars: dict, vy: float):
    """generate action for move_right (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vy: velocity to move along y axis. It is negative when moving right.
    """
    action = local_vars['action']
    action[1] = vy

    return action
