# To terminate this process, ctrl+c is not enough, use "killall python" or something similar!
import pickle
import multiprocessing as mp
import shutil
import os
from os.path import join, exists, isdir, isfile
import mmap

import yaml

from servers import start_server, PipeManager


# Read the configuration file
HOME_DIR = os.environ.get('HOME')
SERVER_CONFIG_PATH = './servers/config.yaml'
with open(SERVER_CONFIG_PATH, 'r') as f:
    server_config = yaml.load(f, Loader=yaml.FullLoader)

START_UP_CONFIG_PATH = './start_up_cfg.yaml'
with open(START_UP_CONFIG_PATH, 'r') as f:
    start_up_config = yaml.load(f, Loader=yaml.FullLoader)

# tmp directory
TMP_PATH = server_config['tmp_path']
SHM_NAME_INTER = server_config['shm_ser_sim']['name']
SHM_SIZE_INTER = server_config['shm_ser_sim']['size']

manager = mp.Manager()

def server_process(*args, **kwargs):
    buffer, = args
    start_server(buffer)
    # the following lines are test code
    # Test getting an image
    # buffer['message'] = ('get_image', 0, SHM_NAME_INTER, SHM_SIZE_INTER)
    # import time
    # time.sleep(5)
    # Test robot moving
    # buffer['message'] = ('move_backward', 0.1)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # time.sleep(3)
    # buffer['message'] = ('move_forward', 0.1)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # time.sleep(3)
    # buffer['message'] = ('move_left', 0.1)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # time.sleep(3)
    # buffer['message'] = ('move_right', 0.1)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # time.sleep(3)
    # buffer['message'] = ('brake',)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # time.sleep(3)
    # buffer['message'] = ('turn_right', 90)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # time.sleep(1)
    # buffer['message'] = ('turn_left', 180)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # Test braking
    # buffer['message'] = ('move_backward', 0.1)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])
    # time.sleep(3)
    # buffer['message'] = ('brake',)
    # while buffer['response'] is None:
    #     pass
    # print(buffer['response'])


def start_simulator():
    bash_components = f'''#!/bin/bash
        source {join(start_up_config["system_cfg"]["CONDA_DIR_PATH"], 'bin/activate')} {start_up_config["system_cfg"]["CONDA_ENV_NAME"]}
        python --version
        python {start_up_config["system_cfg"]["EXECUTE_FILE"]}'''
    bash_path = 'omnigibson/custom_env/start_simulator.sh'
    with open(bash_path, 'w') as f:
        f.write(bash_components)
    os.system(f"bash {bash_path}")


def main():
    # init the pipe, we use pipe to communicate between the simulator process and the server process
    # and the server process will communicate with the high level server
    pipe_manager = PipeManager(
        TMP_PATH, SHM_NAME_INTER, 
        SHM_SIZE_INTER, create=True
    )
    
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
    
    try:
        while True:
            while buffer['message'] is None:
                pass
            response = communicate(
                buffer['message'], pipe_manager)
            buffer['message'] = None
            buffer['response'] = response
    except KeyboardInterrupt:
        simulator_p.terminate()
        server_p.terminate()
        pipe_manager.close(remove=True)


def communicate(message, pipe_manager: PipeManager):
    """
    This is for internal communication between the simulator process 
    and the server process. Here, the 'client' is the server process,
    which forwards the remote client's request to the simulator.
    And the 'server' is the simulator process, which handles the request
    and returns the response to the server process.
    """

    pipe_manager.write_request(message)
    pipe_manager.write_signal(True, role='client')

    while not pipe_manager.read_signal(role='server'):
        pass
    response = pipe_manager.read_request()
    pipe_manager.write_signal(False, role='server')

    return response


if __name__ == '__main__':
    main()
