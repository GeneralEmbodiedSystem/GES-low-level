from servers.utils import SocketManager, PipeManager


def turn_right(socket_manager: SocketManager, pipe: PipeManager):
    """
    Turn the robot right
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

    pipe.write(
        data=("turn_right", angle),
        role="client"
    )
    
    while pipe.read(role="client") is None:
        pass

    # Send the response
    socket_manager.write_request({
        'success': True
    })
