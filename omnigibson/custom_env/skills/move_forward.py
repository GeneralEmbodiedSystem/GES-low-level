"""
Define the move_forward skill.
"""
from servers.utils import PipeManager


def emit_move_forward(pipe: PipeManager, global_states: dict, vel_f: float):
    """Move the robot forward. This function only set a `move_forward` state. 
    the actual movement (generate action at each time) is done in execute_move_forward

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
        vel_f (float): velocity to move forward (`vel_b >= 0`).
    """
    global_states['active_actions']['move_action'] = {
        'func': 'move_forward',
        'args': {
            'vx': vel_f
        }
    }
    # The velocity has been set, so we can send the ack
    pipe.write(
        data="move_forward success",
        role="server"
    )


def execute_move_forward(pipe: PipeManager, global_states: dict, vx: float):
    """generate action for move_forward (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
        vx: velocity to move along x axis. It is positive when moving forward.
    """
    action = global_states['action']
    action[0] = vx

    return action
