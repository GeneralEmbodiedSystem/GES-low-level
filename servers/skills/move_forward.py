from servers.utils import SocketManager, PipeManager


def move_forward(socket_manager: SocketManager, pipe: PipeManager):
    """
    Move the robot forward
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        dist = response_data['dist']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute move
    pipe.write(
        data=("move_forward", dist),
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
