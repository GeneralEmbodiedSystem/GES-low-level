from ..utils.socket_utils import SocketManager


REQ_TEMP = {
    'distance': 0.0
}


def move_x(socket_manager: SocketManager):
    """
    Move the robot in the x direction (front-back)
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