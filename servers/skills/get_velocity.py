from servers.utils import SocketManager, PipeManager


def get_velocity(socket_manager: SocketManager, pipe: PipeManager):
    """
    Get the robot velocity (including direction)
    """
    # Execute move
    pipe.write(
        data=("get_velocity",),
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
