"""
Define the move_backward skill.
"""
import numpy as np

from servers.utils import PipeManager

from omnigibson.utils.transform_utils import quat2euler

from omnigibson.robots import TiagoAdvanced
from omnigibson.custom_env.utils.constant_utils import MOVE_BACKWARD_VEL
from omnigibson.custom_env.utils.eular_utils import calculate_distance


def emit_move_backward(pipe: PipeManager, global_states: dict, dist: float):
    """Move the robot backward. This function only set a `move_backward` state. 
    The actual movement (generate action at each time) is done in execute_move_backward

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        dist (float): distance (in meter) to move backward (`dist > 0`).
    """
    # extra robot
    robot: TiagoAdvanced = global_states['robot']
    
    global_states['active_actions']['move_action'] = {
        'func': 'move_backward',
        'args': {
            'dist': dist
        },
        'init_pos': robot.get_position(),
    }

    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_move_backward(pipe: PipeManager, global_states: dict, dist: float):
    """generate action for move_backward (the actual movement)

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store global variables, which are shared with
            other functions.
        dist (float): distance (in meter) to move backward (`dist > 0`).
    """
    # extract params
    action = global_states['action']
    robot: TiagoAdvanced = global_states['robot']
    init_pos = global_states['active_actions']['move_action']['init_pos']
    # move_exception = global_states['active_actions']['move_exception']

    z_radian = quat2euler(robot.get_orientation())[2]
    x_vel = MOVE_BACKWARD_VEL * np.cos(z_radian)
    y_vel = MOVE_BACKWARD_VEL * np.sin(z_radian)

    if calculate_distance(init_pos, robot.get_position()) <= dist:
        action[0] = x_vel; action[1] = y_vel
    else:
        # Send ack to the server
        pipe.write(
            data="Robot moved backward successfully.",
            role="server"
        )
        # Erase the corresponding active action
        global_states['active_actions']['move_action'] = None
    
    return action
