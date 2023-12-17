import socket

import yaml
import psutil

from servers.utils import SocketManager
from servers.get_image import get_image
from servers.move_x import move_x
from servers.move_y import move_y
from servers.turn import turn


# Functions
functions = [
    get_image,
    move_x,
    move_y,
    turn
]

# Function codes
GET_IMAGE = 0
MOVE_X = 1
MOVE_Y = 2
TURN = 3


def check_port_occupied(port):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1)
            s.bind(('127.0.0.1', port))
            s.listen(1)
            print(f"Port {port} is available.")
            return False
    except OSError as e:
        print(f"Port {port} is already in use.")
        return True


def kill_process_using_port(port):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if proc.info['name'] is None:
                continue
            if f':{port}' in ' '.join(proc.info['cmdline']):
                print(f"Process {proc.info['pid']} is using port {port}. Terminating...")
                proc.terminate()
                print("Process terminated.")
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass


def start_server(pipe: dict):
    """
    Start server on port_number,
    which is used to communicate with High-Level module
    """
    # Read the configuration file
    CONFIG_PATH = './servers/config.yaml'
    with open(CONFIG_PATH, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    base_config = config['tools']['default']
    port_number = base_config['server_address'][1]
    
    if check_port_occupied(port_number):
        kill_process_using_port(port_number)
    
    address = ('localhost', port_number)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(address)
    server_socket.listen(1)

    tmp_path = config['tmp_path']
    socket_manager = SocketManager(tmp_path=tmp_path)

    print("Server listening on", address)

    while True:
        client_socket, client_address = server_socket.accept()
        # print("Connection from", client_address)

        try:
            socket_manager.set_socket(client_socket)
            # Receive the function code
            function_code = socket_manager.recv_func_code()
            if function_code < 0 or function_code >= len(functions):
                print("Invalid function code, received %d" % function_code)
                print("Closing connection.")
                client_socket.close()
                continue
            tool_func = functions[function_code]
            tool_config = config['tools'][tool_func.__name__]
            tool_config = {**base_config, **tool_config}
            # Receive the shared memory name
            if tool_config['shared_memory']:
                shm_name = socket_manager.recv_shared_memory_name()
                socket_manager.init_shared_memory(
                    shm_name, create=False, 
                    size=tool_config['shared_memory_size'])
            # Call the function according to the function code
            tool_func(socket_manager, pipe)
            
        except Exception as e:
            print("Error sending data: ", e)
            print("Closing connection.")
            client_socket.close()
            continue

        # Close the client connection
        client_socket.close()
