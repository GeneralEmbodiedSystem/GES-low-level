from servers.utils import SocketManager


def get_velocity(socket_manager: SocketManager, pipe: dict):
    """
    Get the robot velocity (including direction)
    """
    # Execute move
    pipe['message'] = ("get_velocity",)

    while pipe['response'] is None:
        pass

    # Send the response
    socket_manager.write_request({
        'result': pipe['response']
    })
