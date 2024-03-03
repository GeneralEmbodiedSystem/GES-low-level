from servers.utils import SocketManager, PipeManager


def set_pitch(socket_manager: SocketManager, pipe: PipeManager):
    """
    Set the pitch of the camera of the robot
    """
    try:
        # Receive the request data
        response_data = socket_manager.read_request()
        angle = response_data['angle']

    except Exception as e:
        print("Error receive data: ", e)
        print("Closing connection.")
        return
    
    # Execute action
    pipe.write(
        data=("look_up", angle),
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
