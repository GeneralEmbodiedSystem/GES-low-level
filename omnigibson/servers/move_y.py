from ..utils.socket_utils import SocketManager


REQ_TEMP = {
    'distance': 0.0
}


def move_y(socket_manager: SocketManager):
    """
    Move the robot in the y direction (left-right)
    """
    try:
        # Receive the request data
        distance = socket_manager.recv_float()

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Move ...

    # Send the response
    socket_manager.send_dict({
        'success': True
    })
