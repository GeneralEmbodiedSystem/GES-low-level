from typing import Union, Dict, List

import numpy as np
import yaml

import omnigibson as og
from omnigibson.utils.low_level_utils import convert_ndarray_in_dict_to_shape
from servers import SocketManager, PipeManager
from omnigibson.robots import FetchAdvanced, TiagoAdvanced

SERVER_CONFIG_PATH = './servers/config.yaml'
with open(SERVER_CONFIG_PATH, 'r') as f:
    server_config = yaml.load(f, Loader=yaml.FullLoader)

TMP_PATH = server_config['tmp_path']


def run_get_image(buffer: dict, local_vars: dict,
                  written_len: int, shm_name: str, 
                  shm_size: int):
    # Transfer rgba image to rgb image
    def rgba2rgb(rgba, background=(255,255,255)):
        row, col, ch = rgba.shape
        if ch != 4:
            raise ValueError('RGBA image should have 4 channels.')
        rgb = np.zeros((row, col, 3), dtype='float32')
        r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]
        a = np.asarray(a, dtype='float32') / 255.0
        R, G, B = background
        rgb[:,:,0] = r * a + (1.0 - a) * R
        rgb[:,:,1] = g * a + (1.0 - a) * G
        rgb[:,:,2] = b * a + (1.0 - a) * B

        return np.asarray(rgb, dtype='uint8')
    
    robot: Union[FetchAdvanced, TiagoAdvanced] = local_vars['robot']
    image_res = robot.get_image()
    image_rgb, image_depth = image_res['rgb'], image_res['depth']
    image_rgb = rgba2rgb(image_rgb)
    image_depth = np.expand_dims(image_depth, axis=-1)
    
    # Write the image arrays to shared memory
    socket_manager = SocketManager(
        tmp_path=TMP_PATH
    )
    socket_manager.init_shared_memory(
        shm_name, create=False, size=shm_size
    )
    socket_manager.set_shared_memory_cursor(written_len)
    # We do not send the signal right now
    # We have to wait until the server
    # process receive the ack
    
    # # showing image(test)
    # import matplotlib.pyplot as plt
    # plt.imshow(image_rgb)
    
    socket_manager.write_arrays(
        [image_rgb, image_depth], write=True,
        send_signal=False)
    
    buffer['response'] = 'get_image success'
    og.log.info(convert_ndarray_in_dict_to_shape(image_res))
    # print(convert_ndarray_in_dict_to_shape(image_res))
    # write rgb_image & depth_image to shared memory
    # shared_memory = buffer['shared_memory']
    # ...

