"""
Define the move_backward skill.
"""
from servers.utils import PipeManager


def run_move_backward(pipe: PipeManager, local_vars: dict, vel_b: float):
    """Move the robot backward. This function only set a `move_backward` state. 
    the actual movement (generate action at each time) is done in execute_move_backward

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vel_b (float): velocity to move backward (`vel_b >= 0`).
    """
    local_vars['active_actions']['move_action'] = {
        'func': 'move_backward',
        'args': {
            'vx': -vel_b
        }
    }
    # The velocity has been set, so we can send the ack
    pipe.write(
        data="move_backward success",
        role="server"
    )


def execute_move_backward(pipe: PipeManager, local_vars: dict, vx: float):
    """generate action for move_backward (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vx: velocity to move along x axis. It is negative when moving backward.
    """
    action = local_vars['action']
    action[0] = vx

    return action
