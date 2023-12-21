"""
Define the turn_right skill.
"""


def run_turn_right(buffer: dict, local_vars: dict, angle: int):
    """Turn the robot right. This function only set a `turn_right` state. 
    the actual movement (generate action at each time) is done in execute_turn_right

    Args:
        buffer: a dict used to communicate between the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        angle (int): angle to turn right (`angle` in [1, 180]).
    """
    local_vars['active_actions']['move_action'] = {
        'func': 'turn_right',
        'args': {
            'angle': angle,
        }
    }
    # Reset the steps that have been turned
    local_vars['turn_steps'] = 0
    
    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_turn_right(buffer: dict, local_vars: dict, angle: int):
    """generate action for turn_right (the actual movement)

    Args:
        buffer: a dict used to communicate between the simulator process and the server process.
        local_vars: a dict used to store local variables, which are shared with
            other functions.
        angle (int): angle to turn right (`angle` in [1, 180]).
    """
    # We need to wait for the robot to actually turn
    # So we keep sending the same action until the robot has turned enough
    action = local_vars['action']
    if local_vars['turn_steps'] < angle:
        local_vars['turn_steps'] += 1
        action[2] = -0.341
    else:
        buffer['response'] = 'turn_right success'
        # Erase the corresponding active action
        local_vars['active_actions']['move_action'] = None

    return action
