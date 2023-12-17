from servers.utils import SocketManager


def move_y(socket_manager: SocketManager, pipe: dict):
    """
    Move the robot in the y direction (left-right)
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        distance = response_data['distance']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute move
    pipe['message'] = ("move_y", distance)
    
    while pipe['response'] is None:
        pass
    pipe['response'] = None

    # Send the response
    socket_manager.write_request({
        'success': True
    })
