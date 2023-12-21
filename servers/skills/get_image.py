from servers.utils import SocketManager


def get_image(socket_manager: SocketManager, pipe: dict):
    # Send the response
    written_len = socket_manager.write_request({
        'image_rgb': 0,
        'image_depth': 1
    })

    # Send the message to main process
    pipe['message'] = (
        "get_image", written_len, 
        socket_manager.shm_name, socket_manager.shm_size)
    
    while pipe['response'] is None:
        pass
    pipe['response'] = None
    
    socket_manager.write_arrays(
        [], write=False, send_signal=True)
