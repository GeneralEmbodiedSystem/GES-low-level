# python ./examples/custom/agentExp_version1214/lowLevelServer.py
# To terminate this process, ctrl+c is not enough, use "killall python" or something similar!

import sys
sys.path.append('.')
sys.path.append('..')
sys.path.append('...')
sys.path.append('....')
sys.path.append('.....')

import pickle
import multiprocessing as mp
import subprocess
import shutil
import os
from os.path import join, exists, isdir, isfile
import numpy as np
import time

from utils.low_level_utils import convert_ndarray_in_dict_to_shape

# from omnigibson.servers.server import start_server
from servers.server import start_server

# ROOT_DIR = '~/'
# ISAAC_SIM_VERSION = 'isaac_sim-2022.2.0'
# CONDA_ROOT_DIR = f'{ROOT_DIR}/miniconda3/'
# CONDA_ENV_NAME = 'SIM22'
# CONDA_ENV_VERSION = 'python3.7'
# ISAACSIM_PATH = f'{ROOT_DIR}.local/share/ov/pkg/{ISAAC_SIM_VERSION}'
# SON_PYTHON_PATH = './examples/custom/agentExp_version1214/main.py'
# env_variables = {
#     # "ISAACSIM_PATH": ISAACSIM_PATH,
#     # "ISAACSIM_PYTHON_EXE": f'{ISAACSIM_PATH}/python.sh',
#     "PATH": f"{CONDA_ROOT_DIR}envs/{CONDA_ENV_NAME}/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin",
#     # "PYTHONPATH": f"{CONDA_ROOT_DIR}envs/{CONDA_ENV_NAME}/lib/{CONDA_ENV_VERSION}/site-packages",
#     # "LD_LIBRARY_PATH": "/usr/local/lib",
#     # "LANG": "en_US.UTF-8",
#     # "LC_ALL": "en_US.UTF-8"
# }
# env_vars_dir = './test'
# for env_var_log_file in os.listdir(env_vars_dir):
#     if '.log' in env_var_log_file:
#         with open(join(env_vars_dir, env_var_log_file), 'r') as f:
#             env_var_c = f.read().strip()
#         env_variables[env_var_log_file[:-4]] = env_var_c

# tmpfs directory
HOME_DIR = os.environ.get('HOME')
TMPFS_DIR = join(HOME_DIR, 'tmpfs')

# simulator env
CONDA_ENV_NAME = 'SIM22'
CONDA_DIR_NAME = 'miniconda3' # or anaconda3

manager = mp.Manager()

def server_process(*args, **kwargs):
    # og.log.info(f"args: {args}")
    buffer, = args
    start_server(buffer)
    # buffer['message'] = ('get_image',)
    # buffer['message'] = ('move_x', -0.5)
    # buffer['message'] = ('turn', 90)

def init_tmpfs():
    global TMPFS_DIR
    if not exists(TMPFS_DIR):
        raise ValueError(f"please run \"sudo mount -t tmpfs -o size=512M tmpfs {TMPFS_DIR}\" to create tmpfs in RAM!")
    # delete all files in TMPFS_DIR
    for file in os.listdir(TMPFS_DIR):
        file_path = join(TMPFS_DIR, file)
        if isfile(file_path):
            os.remove(file_path)
        elif isdir(file_path):
            shutil.rmtree(file_path)
        else:
            raise NotImplementedError("The type of the file is not in [file, dir]!")

def start_simulator():
    global CONDA_ENV_NAME, CONDA_DIR_NAME, HOME_DIR
    bash_components = f'''#!/bin/bash
source {join(HOME_DIR, CONDA_DIR_NAME, 'bin/activate')} {CONDA_ENV_NAME}
python --version
python ./examples/custom/agentExp_version1214/main.py'''
    bash_path = './examples/custom/agentExp_version1214/start_simulator.sh'
    with open(bash_path, 'w') as f:
        f.write(bash_components)
    os.system(f"bash {bash_path}")

def main():
    # init the tmpfs
    init_tmpfs()
    
    # start simulator
    simulator_p = mp.Process(target=start_simulator)
    simulator_p.start()

    # start communication with high level server
    global manager
    buffer = manager.dict()
    init_to_None_keys = ['message', 'response']
    for init_to_None_key in init_to_None_keys:
        buffer[init_to_None_key] = None
    server_p = mp.Process(target=server_process, args=(buffer,))
    server_p.start()
    
    while True:
        while buffer['message'] is None:
            pass
        response = communicate(buffer['message'])
        print(f"response: {convert_ndarray_in_dict_to_shape(response)}")
        buffer['message'] = None
        buffer['response'] = response
    
    simulator_p.terminate()
    server_p.terminate()

def communicate(message):
    global TMPFS_DIR
    print(f"message: {message}")
    message_bin = pickle.dumps(message)
    with open(f'{TMPFS_DIR}/message.pkl', 'wb') as f:
        f.write(message_bin)
    
    while not exists(f'{TMPFS_DIR}/response.pkl'):
        pass
    response_path = f'{TMPFS_DIR}/response.pkl'
    with open(response_path, 'rb') as f:
        response_bin = f.read()
    os.remove(response_path)
    response = pickle.loads(response_bin)
    return response

if __name__ == '__main__':
    main()