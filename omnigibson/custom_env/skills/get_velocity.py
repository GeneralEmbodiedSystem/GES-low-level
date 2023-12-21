"""
Define the get_velocity skill.
"""


def run_get_velocity(buffer: dict, local_vars: dict):
    """Get the velocity (including direction) of the robot. 

    Args:
        buffer: a dict used to communicate between the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
    """
    move_action = local_vars['active_actions']['move_action']
    if move_action is None:
        buffer['response'] = "The robot is still."
    else:
        buffer['response'] = "The robot is moving " + \
            move_action['func'][5:] + " with velocity " + \
            str(abs(list(move_action['args'].values())[0])) + ' m/s.'
