import sys
sys.path.append('..')

from utils.socket_utils import SocketManager


def get_image(socket_manager: SocketManager, pipe: dict):
    # Receive the shared memory name
    shm_name = socket_manager.recv_shared_memory_name()
    socket_manager.init_shared_memory(shm_name, create=False)
    
    # Send the message to main process
    pipe['message'] = ("get_image",)
    
    while pipe['response'] is None:
        pass
    image_rgb = pipe['response']['image_rgb']
    image_depth = pipe['response']['image_depth']
    pipe['response'] = None
    
    # Send the response
    socket_manager.send_dict({
        'image_rgb': image_rgb,
        'image_depth': image_depth
    })
