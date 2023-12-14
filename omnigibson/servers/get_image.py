from ..utils.socket_utils import SocketManager


def get_image(socket_manager: SocketManager):
    # Send the response
    socket_manager.send_dict({
        'image': None
    })
