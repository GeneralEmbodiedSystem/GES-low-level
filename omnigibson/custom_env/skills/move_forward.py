"""
Define the move_forward skill.
"""


def run_move_forward(buffer: dict, local_vars: dict, vel_f: float):
    """Move the robot forward. This function only set a `move_forward` state. 
    the actual movement (generate action at each time) is done in execute_move_forward

    Args:
        buffer: a dict used to communicate between the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vel_f (float): velocity to move forward (`vel_b >= 0`).
    """
    local_vars['active_actions']['move_action'] = {
        'func': 'move_forward',
        'args': {
            'vx': vel_f
        }
    }
    # The velocity has been set, so we can send the ack
    buffer['response'] = 'move_forward success'


def execute_move_forward(buffer: dict, local_vars: dict, vx: float):
    """generate action for move_forward (the actual movement)

    Args:
        buffer: a dict used to communicate between the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        vx: velocity to move along x axis. It is positive when moving forward.
    """
    action = local_vars['action']
    action[0] = vx

    return action
