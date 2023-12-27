from servers.utils import SocketManager, PipeManager


def turn_left(socket_manager: SocketManager, pipe: PipeManager):
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

    pipe.write(
        data=("turn_left", angle),
        role="client"
    )
    
    while True:
        data = pipe.read(role="client")
        if data is not None:
            break

    # Send the response
    socket_manager.write_request({
        'result': data
    })
