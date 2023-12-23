# To terminate this process, ctrl+c is not enough, use "killall python" or something similar!
import multiprocessing as mp
import os
from os.path import join
import sys
import select

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


def start_simulator():
    bash_components = f'''#!/bin/bash
        source {join(start_up_config["system_cfg"]["CONDA_DIR_PATH"], 'bin/activate')} {start_up_config["system_cfg"]["CONDA_ENV_NAME"]}
        python --version
        python {start_up_config["system_cfg"]["EXECUTE_FILE"]} --config_file {start_up_config["system_cfg"]["ENV_CONFIG_FILE"]}'''
    bash_path = 'omnigibson/custom_env/start_simulator.sh'
    with open(bash_path, 'w') as f:
        f.write(bash_components)
    os.system(f"bash {bash_path}")


def main():
    # This is for internal communication between the simulator process 
    # and the server process. Here, the 'client' is the server process,
    # which forwards the remote client's request to the simulator.
    # And the 'server' is the simulator process, which handles the request
    # and returns the response to the server process.
    pipe_manager = PipeManager(
        TMP_PATH, SHM_NAME_INTER, 
        SHM_SIZE_INTER, create=True
    )
    
    # start simulator
    simulator_p = mp.Process(target=start_simulator)
    simulator_p.start()

    # start communication with high level server
    try:
        # start_server(pipe_manager)

        # ------------------ Start Test Skills ------------------
        pipe = pipe_manager
        
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:  # If there is input to read
                user_input = sys.stdin.readline().strip()
                print("your operation is:", user_input)
                """skill is like: 
                ("move_forward", 0.1)
                ("move_backward", 0.1)
                ("brake", )
                ("turn_left", 90)
                ("turn_right", 90)
                ("move_left", 0.1)
                ("move_right", 0.1)
                ("get_velocity",)
                """
                pipe.write(
                    data=eval(user_input),
                    role="client"
                )
            
                while True:
                    data = pipe_manager.read(role="client")
                    if data is not None:
                        print(data)
                        break
        # ------------------ End Test Skills ------------------

    except KeyboardInterrupt:
        simulator_p.terminate()
        pipe_manager.close(remove=True)


if __name__ == '__main__':
    main()
