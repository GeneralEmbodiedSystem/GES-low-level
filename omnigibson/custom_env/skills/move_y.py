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


def run_move_y(buffer: dict, local_vars: dict, sy: float):
    """distance to move along y axis. this function only set a `move_y` state. 
    the actual movement(generate action) is done in execute_move_y

    Args:
        s (int): distance to move along y axis. [-1, -0.1] means move right. [0.1, 1] means move left.
        cur_step (int): _description_
    """
    env, robot = local_vars['env'], local_vars['robot']
    
    init_y = robot.get_position()[1]
    left_or_right = 1 if sy > 0 else -1
    act_y = left_or_right * 0.2  # 0.2 is the velocity 
    
    keys = ['init_y', 'act_y', 'left_or_right', 'sy']
    for key in keys:
        local_vars[key] = eval(key)
    local_vars['keys_y'] = keys

    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_move_y(buffer: dict, local_vars: dict):
    """generate action for move_y (the actual movement), including reset the `move_y` state

    Args:
        buffer (dict): _description_
        local_vars (dict): _description_
    """
    env, robot = local_vars['env'], local_vars['robot']
    action = local_vars['action']
    
    if 'act_y' in local_vars:
        left_or_right, init_y, sy = local_vars['left_or_right'], local_vars['init_y'], local_vars['sy']
        if left_or_right * (robot.get_position()[1] - init_y) >= abs(sy):
            for key in local_vars['keys_y']:
                local_vars.pop(key)
            local_vars.pop('keys_y')
            buffer['response'] = 'move_y success'
    if 'act_y' in local_vars:
        action[1] = local_vars['act_y']
    return action
