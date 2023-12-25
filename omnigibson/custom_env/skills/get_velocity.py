"""
Define the get_velocity skill.
"""
from servers.utils import PipeManager
from omnigibson.custom_env.exceptions import MoveException


def emit_get_velocity(pipe: PipeManager, global_states: dict):
    """Get the velocity (including direction) of the robot. 

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
    """
    move_action = global_states['active_actions']['move_action']
    if move_action is None:
        pipe.write(
            data="The robot is still.",
            role="server"
        )
    elif isinstance(move_action, MoveException):
        pipe.write(
            data=str(move_action),
            role="server"
        )
    else:
        data = "The robot is moving " + \
            move_action['func'][5:] + " with velocity " + \
            str(abs(list(move_action['args'].values())[0])) + ' m/s.'
        pipe.write(
            data=data,
            role="server"
        )
