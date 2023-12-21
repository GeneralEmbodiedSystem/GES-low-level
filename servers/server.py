import socket
import time
import yaml
import psutil

from servers.utils import SocketManager
from servers.skills import get_image, move_forward, move_backward, \
    move_left, move_right, brake, turn_left, turn_right



# Functions
functions = [
    get_image,
    move_forward,
    move_backward,
    move_left,
    move_right,
    brake,
    turn_left,
    turn_right
]

# Function codes
GET_IMAGE = 0
MOVE_FORWARD = 1
MOVE_BACKWARD = 2 
MOVE_LEFT = 3
MOVE_RIGHT = 4
BRAKE = 5
TURN_LEFT = 6
TURN_RIGHT = 7


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
    for conn in psutil.net_connections(kind='inet'):
        if conn.laddr.port == port:
            try:
                proc = psutil.Process(conn.pid)
                print(f"Process {proc.pid} is using port {port}. Terminating...")
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
        time.sleep(2)
    
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
            if tool_config is None:
                tool_config = base_config
            else:
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
