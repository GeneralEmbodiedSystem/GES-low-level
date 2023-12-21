from servers.utils import SocketManager


def brake(socket_manager: SocketManager, pipe: dict):
    """
    Brake the robot
    """
    # Execute move
    pipe['message'] = ("brake",)
    
    while pipe['response'] is None:
        pass
    pipe['response'] = None

    # Send the response
    socket_manager.write_request({
        'success': True
    })
