from servers.utils import SocketManager, PipeManager


def get_image(socket_manager: SocketManager, pipe: PipeManager):
    # Send the response
    written_len = socket_manager.write_request({
        'image_rgb': 0,
        'image_depth': 1
    })

    # Send the message to main process
    pipe.write(
        data=("get_image", written_len, 
            socket_manager.shm_name, socket_manager.shm_size),
        role="client"
    )
    
    while pipe.read(role="client") is None:
        pass
    
    socket_manager.write_arrays(
        [], write=False, send_signal=True)
