"""
Define the move_right skill.
"""
from servers.utils import PipeManager

from omnigibson.custom_env.utils.constant_utils import MOVE_RIGHT_VEL



def emit_move_right(pipe: PipeManager, global_states: dict, dist: float):
    """Move the robot right. This function only set a `move_right` state. 
    The actual movement (generate action at each time) is done in execute_move_right

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        dist (float): distance (in meter) to move right (`dist > 0`).
    """
    robot = global_states['robot']
    global_states['active_actions']['move_action'] = {
        'func': 'move_right',
        'args': {
            'dist': dist
        }
    }
    # Current position of the robot
    global_states['init_y'] = robot.get_position()[1]

    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_move_right(pipe: PipeManager, global_states: dict, dist: float):
    """generate action for move_right (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        dist (float): distance (in meter) to move right (`dist > 0`).
    """
    action = global_states['action']
    robot = global_states['robot']
    init_y = global_states['init_y']
    move_exception = global_states['active_actions']['move_exception']

    if abs(robot.get_position()[1] - init_y) <= dist:
        action[1] = MOVE_RIGHT_VEL
    else:
        pipe.write(
            data="move_right success",
            role="server"
        )
        # Erase the corresponding active action
        global_states['active_actions']['move_action'] = None
    
    return action
