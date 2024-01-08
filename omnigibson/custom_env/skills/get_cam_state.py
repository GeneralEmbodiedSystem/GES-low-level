from typing import Union

import numpy as np
import yaml

import omnigibson as og
from servers import SocketManager, PipeManager
from omnigibson.robots import FetchAdvanced, TiagoAdvanced


# Load the config file
SERVER_CONFIG_PATH = './servers/config.yaml'
with open(SERVER_CONFIG_PATH, 'r') as f:
    server_config = yaml.load(f, Loader=yaml.FullLoader)
TMP_PATH = server_config['tmp_path']


def emit_get_cam_state(pipe: PipeManager, global_states: dict):
    
    robot: Union[FetchAdvanced, TiagoAdvanced] = global_states['robot']
    obs_res = robot.get_cam_obs()
    
    print(obs_res['camera'])
    
    pipe.write(
        data="get_cam_state success",
        role="server"
    )
