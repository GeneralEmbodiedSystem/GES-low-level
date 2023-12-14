import socket

from ..utils.socket_utils import SocketManager
from .get_image import get_image
from .move_x import move_x
from .move_y import move_y

# Functions
functions = [
    get_image,
    move_x,
    move_y
]

# Function codes
GET_IMAGE = 0
MOVE_X = 1
MOVE_Y = 2


def start_server(port_number, shared_memory_size):
    """
    Start server on port_number,
    which is used to communicate with High-Level module
    """
    address = ('localhost', port_number)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(address)
    server_socket.listen(1)
    socket_manager = SocketManager(True, shared_memory_size)

    print("Server listening on", address)

    while True:
        client_socket, client_address = server_socket.accept()
        # print("Connection from", client_address)

        try:
            socket_manager.set_socket(client_socket)
            # Receive the shared memory name
            shm_name = socket_manager.recv_shared_memory_name()
            socket_manager.init_shared_memory(shm_name, create=False)

            # Receive the function code
            function_code = socket_manager.recv_int()
            # Call the function according to the function code
            functions[function_code](socket_manager)
            
            if function_code < 0 or function_code >= len(functions):
                print("Invalid function code, received %d" % function_code)
                print("Closing connection.")
                client_socket.close()
                continue
            
        except Exception as e:
            print("Error sending data: ", e)
            print("Closing connection.")
            client_socket.close()
            continue

        # Close the client connection
        client_socket.close()
