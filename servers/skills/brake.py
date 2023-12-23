from servers.utils import SocketManager, PipeManager


def brake(socket_manager: SocketManager, pipe: PipeManager):
    """
    Brake the robot
    """
    # Execute move
    pipe.write(
        data=("brake",),
        role="client"
    )
    
    while pipe.read(role="client") is None:
        pass

    # Send the response
    socket_manager.write_request({
        'success': True
    })
