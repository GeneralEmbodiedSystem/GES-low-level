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


def run_turn(buffer: dict, local_vars: dict, deg: float):
    """degrees to rotate base. this function only set a `turn` state. 
    the actual movement(generate action) is done in execute_turn

    Args:
        deg (int): -180 ~ 180. [-180, -1] means turn right. [1, 180] means turn left.
        cur_step (int): current step (start time)
    """
    left_or_right = 1 if deg > 0 else -1  # 1 means turn left, -1 means turn right
    act_turn = left_or_right * 0.341  # 1 degrees per step
    continue_steps = abs(int(deg))
    
    keys = ['act_turn', 'continue_steps']
    for key in keys:
        local_vars[key] = eval(key)
    local_vars['keys_turn'] = keys
    
    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def execute_turn(buffer: dict, local_vars: dict):
    """generate action for turn body (the actual movement), including reset the `turn` state

    Args:
        buffer (dict): _description_
        local_vars (dict): _description_
    """
    env, robot = local_vars['env'], local_vars['robot']
    action = local_vars['action']
    
    if 'act_turn' in local_vars:
        if local_vars['continue_steps'] <= 0:
            for key in local_vars['keys_turn']:
                local_vars.pop(key)
            local_vars.pop('keys_turn')
            buffer['response'] = 'turn success'
        else:
            local_vars['continue_steps'] -= 1
    if 'act_turn' in local_vars:
        action[2] = local_vars['act_turn']
    return action