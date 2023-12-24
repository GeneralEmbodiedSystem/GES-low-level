"""
Define the brake skill.
"""
from servers.utils import PipeManager


def emit_brake(pipe: PipeManager, local_vars: dict):
    """Brake the robot. This function will eliminate all the active move actions. 
    So the velocity will be set to zero immediately.

    Args:
        pipe: a wrapper of shared memory used to communicate between 
            the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
    """
    local_vars['active_actions']['move_action'] = None
    # The brake has been set, so we can send the ack
    pipe.write(
        data="brake success",
        role="server"
    )
