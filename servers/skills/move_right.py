from servers.utils import SocketManager


# Define the speed modes
WALK_SPEED = 0.2
CRAWL_SPEED = 0.05


def move_right(socket_manager: SocketManager, pipe: dict):
    """
    Move the robot right
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        mode = response_data['mode']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute move
    if mode == 'walk':
        velocity = WALK_SPEED
    elif mode == 'crawl':
        velocity = CRAWL_SPEED
    else:
        print("Invalid speed mode: %s" % mode)
        print("Closing connection.")
        return
    pipe['message'] = ("move_right", velocity)
    
    while pipe['response'] is None:
        pass
    pipe['response'] = None

    # Send the response
    socket_manager.write_request({
        'success': True
    })
