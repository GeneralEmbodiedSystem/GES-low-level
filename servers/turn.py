import sys
sys.path.append('..')

from servers.utils import SocketManager


REQ_TEMP = {
    'angle': 0
}


def turn(socket_manager: SocketManager, pipe: dict):
    """
    Turn the robot (left or right)
    """
    try:
        # Receive the request data
        response_data = socket_manager.recv_dict(REQ_TEMP)
        angle = response_data['angle']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute turn
    pipe['message'] = ("turn", angle)
    
    while pipe['response'] is None:
        pass
    pipe['response'] = None

    # Send the response
    socket_manager.send_dict({
        'success': True
    })