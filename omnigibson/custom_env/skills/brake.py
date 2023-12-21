"""
Define the brake skill.
"""


def run_brake(buffer: dict, local_vars: dict):
    """Brake the robot. This function will eliminate all the active move actions. 
    So the velocity will be set to zero immediately.

    Args:
        buffer: a dict used to communicate between the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
    """
    local_vars['active_actions']['move_action'] = None
    # The brake has been set, so we can send the ack
    buffer['response'] = 'brake success'
