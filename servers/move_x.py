import sys
sys.path.append('..')

from servers.utils import SocketManager


REQ_TEMP = {
    'distance': 0.0
}


def move_x(socket_manager: SocketManager, pipe: dict):
    """
    Move the robot in the x direction (front-back)
    """
    try:
        # Receive the request data
        response_data = socket_manager.recv_dict(REQ_TEMP)
        distance = response_data['distance']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute move
    pipe['message'] = ("move_x", distance)
    
    while pipe['response'] is None:
        pass
    pipe['response'] = None

    # Send the response
    socket_manager.send_dict({
        'success': True
    })