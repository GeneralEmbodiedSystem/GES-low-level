from servers.utils import SocketManager


def turn_left(socket_manager: SocketManager, pipe: dict):
    """
    Turn the robot left
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        angle = response_data['angle']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute move
    pipe['message'] = ("turn_left", angle)
    
    while pipe['response'] is None:
        pass
    pipe['response'] = None

    # Send the response
    socket_manager.write_request({
        'success': True
    })
