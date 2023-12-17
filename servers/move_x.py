from servers.utils import SocketManager


def move_x(socket_manager: SocketManager, pipe: dict):
    """
    Move the robot in the x direction (front-back)
    """
    try:
        # Receive the distance
        response_data = socket_manager.read_request()
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
    socket_manager.write_request({
        'success': True
    })
