"""
Define the move_forward skill.
"""
from servers.utils import PipeManager

import omnigibson.utils.transform_utils as T

from omnigibson.custom_env.utils.constant_utils import MOVE_FORWARD_VEL


def emit_move_forward(pipe: PipeManager, global_states: dict, dist: float):
    """Move the robot forward. This function only set a `move_forward` state. 
    The actual movement (generate action at each time) is done in execute_move_forward

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        dist (float): distance (in meter) to move forward (`dist > 0`).
    """
    robot = global_states['robot']
    global_states['active_actions']['move_action'] = {
        'func': 'move_forward',
        'args': {
            'dist': dist
        }
    }
    # Current position of the robot
    global_states['init_x'] = robot.get_position()[0]

    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_move_forward(pipe: PipeManager, global_states: dict, dist: float):
    """generate action for move_forward (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        dist (float): distance (in meter) to move forward (`dist > 0`).
    """
    action = global_states['action']
    robot = global_states['robot']
    init_x = global_states['init_x']
    move_exception = global_states['active_actions']['move_exception']

    if abs(robot.get_position()[0] - init_x) <= dist:
        action[0] = MOVE_FORWARD_VEL
    else:
        pipe.write(
            data="move_forward success",
            role="server"
        )
        # Erase the corresponding active action
        global_states['active_actions']['move_action'] = None

    return action
