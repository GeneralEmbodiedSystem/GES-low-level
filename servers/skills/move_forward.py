from servers.utils import SocketManager, PipeManager


# Define the speed modes
RUN_SPEED = 0.3
WALK_SPEED = 0.1
CRAWL_SPEED = 0.05


def move_forward(socket_manager: SocketManager, pipe: PipeManager):
    """
    Move the robot forward
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
    if mode == 'run':
        velocity = RUN_SPEED
    elif mode == 'walk':
        velocity = WALK_SPEED
    elif mode == 'crawl':
        velocity = CRAWL_SPEED
    else:
        print("Invalid speed mode: %s" % mode)
        print("Closing connection.")
        return
    
    pipe.write(
        data=("move_forward", velocity),
        role="client"
    )
    
    while pipe.read(role="client") is None:
        pass

    # Send the response
    socket_manager.write_request({
        'success': True
    })
