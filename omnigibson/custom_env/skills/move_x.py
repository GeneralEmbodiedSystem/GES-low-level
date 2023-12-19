from typing import Union, Dict, List

import numpy as np
import yaml

import omnigibson as og
from omnigibson.utils.low_level_utils import convert_ndarray_in_dict_to_shape
from servers import SocketManager, PipeManager
from omnigibson.robots import FetchAdvanced, TiagoAdvanced

SERVER_CONFIG_PATH = './servers/config.yaml'
with open(SERVER_CONFIG_PATH, 'r') as f:
    server_config = yaml.load(f, Loader=yaml.FullLoader)

TMP_PATH = server_config['tmp_path']

def run_move_x(buffer: dict, local_vars: dict, sx: float):
    """distance to move along x axis. this function only set a `move_x` state. 
    the actual movement(generate action at each time) is done in execute_move_x

    Args:
        s (int): distance to move along x axis. [-1, -0.1] means move back. [0.1, 1] means move forward.
        cur_step (int): _description_
    """
    env, robot = local_vars['env'], local_vars['robot']
    
    init_x = robot.get_position()[0]
    forward_or_back = 1 if sx > 0 else -1
    act_x = forward_or_back * 0.1  # you can modify here: 0.1 is the velocity 
    
    keys = ['init_x', 'act_x', 'forward_or_back', 'sx']
    for key in keys:
        local_vars[key] = eval(key)  # init_x, act_x, forward_or_back, sx
    local_vars['keys_x'] = keys

    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now
    
def execute_move_x(buffer: dict, local_vars: dict):
    """generate action for move_x (the actual movement), including reset the `move_x` state

    Args:
        buffer (dict): _description_
        local_vars (dict): _description_
    """
    env, robot = local_vars['env'], local_vars['robot']
    action = local_vars['action']
    
    if 'act_x' in local_vars:
        forward_or_back, init_x, sx = local_vars['forward_or_back'], local_vars['init_x'], local_vars['sx']
        if forward_or_back * (robot.get_position()[0] - init_x) >= abs(sx):  # you can modify here
            for key in local_vars['keys_x']:
                local_vars.pop(key)
            local_vars.pop('keys_x')
            # The robot has now moved, so we can send the ack
            buffer['response'] = 'move_x success'  # you can modify here
    # og.log.info(f"act_x2: {'act_x' in local_vars}")
    if 'act_x' in local_vars:
        action[0] = local_vars['act_x']  # you can modify here
    return action