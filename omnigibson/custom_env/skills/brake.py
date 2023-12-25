"""
Define the brake skill.
"""
from servers.utils import PipeManager


def emit_brake(pipe: PipeManager, global_states: dict):
    """Brake the robot. This function will eliminate all the active move actions. 
    So the velocity will be set to zero immediately.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        global_states: a dict used to store local variables, which are shared with
            other functions.
    """
    global_states['active_actions']['move_action'] = None
    # The brake has been set, so we can send the ack
    pipe.write(
        data="brake success",
        role="server"
    )
