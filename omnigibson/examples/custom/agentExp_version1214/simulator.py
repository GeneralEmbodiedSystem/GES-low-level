# -*- coding: UTF-8 -*-

# python ./examples/custom/agentExp_version1214/main.py

# mount tmpfs:
# mkdir ~/tmpfs
# sudo mount -t tmpfs -o size=512M tmpfs ~/tmpfs/

"""
Example script demo'ing robot manipulation control with grasping.
"""
import os
from os.path import exists, join
# os.system("bash -c 'source /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/setup_conda_env.sh'")

import sys
from math import ceil
from typing import Union, Dict, List

import pickle
import numpy as np
import multiprocessing as mp
import yaml

import omnigibson as og
from omnigibson.utils.low_level_utils import convert_ndarray_in_dict_to_shape, omit_convert_dict
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from omnigibson.utils.ui_utils import choose_from_options, KeyboardRobotController
from omnigibson.utils.asset_utils import get_available_g_scenes, get_available_og_scenes
from omnigibson.robots import FetchAdvanced, TiagoAdvanced
from servers import SocketManager, PipeManager

from omnigibson.utils.ui_utils import create_module_logger
log = create_module_logger(module_name=__name__)


GRASPING_MODES = dict(
    sticky="Sticky Mitten - Objects are magnetized when they touch the fingers and a CLOSE command is given",
    physical="Physical Grasping - No additional grasping assistance applied",
)

# Don't use GPU dynamics and Use flatcache for performance boost
gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True

# Read the configuration file
CONFIG_PATH = './servers/config.yaml'
with open(CONFIG_PATH, 'r') as f:
    config = yaml.load(f, Loader=yaml.FullLoader)
TMP_PATH = config['tmp_path']
SHM_NAME_INTER = config['shm_ser_sim']['name']
SHM_SIZE_INTER = config['shm_ser_sim']['size']


def pipe_read(pipe_manager: PipeManager):
    while not pipe_manager.read_signal(role='client'):
        pass
    message = pipe_manager.read_request()
    pipe_manager.write_signal(False, role='client')
    
    return message


def pipe_write(pipe_manager: PipeManager, data):
    pipe_manager.write_request(data)
    pipe_manager.write_signal(True, role='server')


def communicate_process(*args, **kwargs):
    """
    This is for internal communication between the simulator process 
    and the server process. Here, the 'client' is the server process,
    which forwards the remote client's request to the simulator.
    And the 'server' is the simulator process, which handles the request
    and returns the response to the server process.
    """
    # og.log.info(f"args: {args}")
    buffer, = args
    pipe_manager = PipeManager(
        TMP_PATH, SHM_NAME_INTER, 
        SHM_SIZE_INTER, create=False
    )
    
    while True:
        message = pipe_read(pipe_manager)
        buffer['message'] = message
        while buffer['response'] is None:
            pass
        pipe_write(pipe_manager, buffer['response'])
        buffer['response'] = None


def main(random_selection=False, headless=False, short_exec=False):
    """
    Robot grasping mode demo with selection
    Queries the user to select a type of grasping mode
    """
    # start server process
    manager = mp.Manager()
    buffer = manager.dict()
    init_to_None_keys = ['message', 'response']
    for init_to_None_key in init_to_None_keys:
        buffer[init_to_None_key] = None
    P = mp.Process(target=communicate_process, args=(buffer,))
    P.start()
    
    # og.log.info(f"Demo {__file__}\n    " + "*" * 80 + "\n    Description:\n" + main.__doc__ + "*" * 80)

    # Choose type of grasping
    # grasping_mode = choose_from_options(options=GRASPING_MODES, name="grasping mode", random_selection=random_selection)
    grasping_mode = 'sticky'
    print(f"grasping_mode: {grasping_mode}")

    # Create environment configuration to use
    scene_type = "InteractiveTraversableScene"  # fix
    scenes = get_available_og_scenes() if scene_type == "InteractiveTraversableScene" else get_available_g_scenes()
    # scene_model = choose_from_options(options=scenes, name="scene model", random_selection=random_selection)
    scene_model = 'Beechwood_0_int'
    
    print(f"scene model: {scene_model}")
    scene_cfg = dict(
        type=scene_type,
        scene_model=scene_model,
        load_object_categories=["floors", "walls", "ceilings"],
    )
    # if scene_type == "InteractiveTraversableScene":  # 这里我直接默认full了
    #     load_options = {
    #         "Quick": "Only load the building assets (i.e.: the floors, walls, ceilings)",
    #         "Full": "Load all interactive objects in the scene",
    #     }
    #     load_mode = choose_from_options(options=load_options, name="load mode", random_selection=random_selection)
    #     if load_mode == "Quick":
    #         scene_cfg["load_object_categories"] = ["floors", "walls", "ceilings"]
    
    robot0_cfg = dict(
        # type="Fetch",
        type="TiagoAdvanced",
        obs_modalities=["scan", "rgb", "depth"],
        action_type="continuous",
        action_normalize=True,
        grasping_mode=grasping_mode,
        position=[-4.0, 0.0, 0.0],
    )

    # Define objects to load
    # 整体上x 和 y小一些，可以让robot和物体放到客厅里去。现在的房间太小了
    objects_cfg = [ 
        dict(
            type="DatasetObject",
            name="delicious_apple",
            category="apple",
            model="agveuv",
            size=0.07,
            position=[0.83, -0.1, 0.97],
        ),  # apple
        dict(
            type="DatasetObject",
            name="alarm_clock",
            category="alarm_clock",
            model="cvknrh",
            size=0.06,
            position=[1.53, 0.1, 0.97],
        ),  # alarm_clock
        dict(
            type="DatasetObject",
            name="water_bottle",
            category="water_bottle",
            model="ackxiy",
            size=0.06,
            position=[1.33, -0.5, 0.97],
        ),  # alarm_clock
        {
            "type": "LightObject", 
            "name": "brilliant_light",
            "light_type": "Sphere", 
            "intensity": 50000, 
            "radius": 0.1, 
            "position": [3.0, 3.0, 4.0],
        },
        dict(
            type="DatasetObject",
            name="table",
            category="breakfast_table",
            model="lcsizg",
            bounding_box=[0.5, 0.5, 0.8],
            fit_avg_dim_volume=False,
            fixed_base=True,
            position=[0.7, -0.1, 0.6],
            orientation=[0, 0, 0.707, 0.707],
        ),  # table
        # dict(
        #     type="DatasetObject",
        #     name="chair",
        #     category="straight_chair",
        #     model="amgwaw",
        #     bounding_box=None,
        #     fit_avg_dim_volume=True,
        #     fixed_base=False,
        #     position=[0.45, 0.65, 0.425],
        #     orientation=[0, 0, -0.9990215, -0.0442276],
        # ),  # chair
        dict(
            type="PrimitiveObject",
            name="box",
            primitive_type="Cube",
            rgba=[1.0, 0, 0, 1.0],
            size=0.05,
            position=[0.53, -0.1, 0.97],
        ),  # box
    ]

    # Compile config
    cfg = dict(scene=scene_cfg, robots=[robot0_cfg], objects=objects_cfg)

    # Create the environment
    env = og.Environment(configs=cfg, action_timestep=1/60., physics_timestep=1/60.)

    # Reset the robot
    robot = env.robots[0]

    robot.set_position([0, -0.5, 0])
    robot.reset()
    # robot.keep_still()

    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = 720
            sensor.image_width = 720
    
    # log.warning(f"vision_res: {omit_convert_dict(robot.get_image())}")
    
    # return

    # Update the simulator's viewer camera's pose so it points towards the robot
    og.sim.viewer_camera.set_position_orientation(
        position=np.array([1.20465,  1.51194,  2.14789]),
        orientation=np.array([0.144,  0.424,  0.846, 0.28757]),
    )

    # Create teleop controller
    action_generator = KeyboardRobotController(robot=robot)

    # Print out relevant keyboard info if using keyboard teleop
    action_generator.print_keyboard_teleop_info()

    # Other helpful user info
    print("Running demo with grasping mode {}.".format(grasping_mode))
    print("Press ESC to quit")

    # Loop control until user quits
    short_exec = True
    max_steps = -1 if not short_exec else 600
    step = 0
    action_random_selection = False
    
    while step != max_steps:
        action = np.zeros(robot.action_dim)
        execute_message(buffer, locals())
        generate_action(buffer, locals())
        env.step(action)

    """
    TiagoAdvance action idx:
    0: base move (x axis), + means forward and - means back. 0.1 * 84 steps ~ 0.2m.  0.1 * 1 step ~ [0.00238, 0.00244]m
    1: base move (y axis), + means left and - means right. 0.1 * 50 steps ~ 0.1173m.  0.1 * 1 step ~ 0.002346 m
    2: base rotate (+left/-right). 614 steps * 0.2 = 122.8 ~ 360 degree left;  61.4 ~ 180 degree left;
        so 90 degree left is 30.7 or 0.1 * 307 steps; or 0.6 * 51 steps (action[2]=0.6 is large enough!)
    3: head rotate (left/right)
    5: left hand front/back
    """

    # Always shut down the environment cleanly at the end
    env.close()
    P.terminate()


def execute_message(buffer: dict, local_vars: dict):
    '''
    @param message(tuple): (CMD, *args)
        (get_image, ...)
        (move_x, distance)
        (move_y, distance)
        (moverz, rotate degree)
    '''
    if buffer['message'] is None:
        return
    message = buffer['message']
    buffer['message'] = None
    
    message_execute_func_mapping = {
        'get_image': run_get_image,
        'move_x': run_move_x,
        'move_y': run_move_y,
        'turn': run_turn,
    }
    og.log.info(f"message: {message}")
    message_execute_func_mapping[message[0]](buffer, local_vars, *message[1:])


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
    socket_manager.write_arrays(
        [image_rgb, image_depth], write=True,
        send_signal=False)
    
    buffer['response'] = 'get_image success'
    og.log.info(convert_ndarray_in_dict_to_shape(image_res))
    # print(convert_ndarray_in_dict_to_shape(image_res))
    # write rgb_image & depth_image to shared memory
    # shared_memory = buffer['shared_memory']
    # ...


def run_move_x(buffer: dict, local_vars: dict, sx: float):
    """distance to move along x axis

    Args:
        s (int): distance to move along x axis. [-1, -0.1] means move back. [0.1, 1] means move forward.
        cur_step (int): _description_
    """
    env, robot = local_vars['env'], local_vars['robot']
    
    init_x = robot.get_position()[0]
    forward_or_back = 1 if sx > 0 else -1
    act_x = forward_or_back * 0.1
    
    keys = ['init_x', 'act_x', 'forward_or_back', 'sx']
    for key in keys:
        local_vars[key] = eval(key)
    local_vars['keys_x'] = keys

    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def run_move_y(buffer: dict, local_vars: dict, sy: float):
    """distance to move along y axis

    Args:
        s (int): distance to move along y axis. [-1, -0.1] means move right. [0.1, 1] means move left.
        cur_step (int): _description_
    """
    env, robot = local_vars['env'], local_vars['robot']
    
    init_y = robot.get_position()[1]
    left_or_right = 1 if sy > 0 else -1
    act_y = left_or_right * 0.2
    
    keys = ['init_y', 'act_y', 'left_or_right', 'sy']
    for key in keys:
        local_vars[key] = eval(key)
    local_vars['keys_y'] = keys

    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def run_turn(buffer: dict, local_vars: dict, deg: float):
    """degrees to rotate base

    Args:
        deg (int): -180 ~ 180. [-180, -1] means turn right. [1, 180] means turn left.
        cur_step (int): current step (start time)
    """
    left_or_right = 1 if deg > 0 else -1  # 1 means turn left, -1 means turn right
    act_turn = left_or_right * 0.341  # 1 degrees per step
    continue_steps = abs(int(deg))
    
    keys = ['act_turn', 'continue_steps']
    for key in keys:
        local_vars[key] = eval(key)
    local_vars['keys_turn'] = keys
    
    # We have now send the command to the simulation,
    # but we need to wait for the robot to actually move
    # So we do not send the ack right now


def generate_action(buffer: dict, local_vars: dict):
    env, robot = local_vars['env'], local_vars['robot']
    action = local_vars['action']
    
    # og.log.info(f"act_x1: {'act_x' in local_vars}")
    if 'act_x' in local_vars:
        forward_or_back, init_x, sx = local_vars['forward_or_back'], local_vars['init_x'], local_vars['sx']
        if forward_or_back * (robot.get_position()[0] - init_x) >= abs(sx):
            for key in local_vars['keys_x']:
                local_vars.pop(key)
            local_vars.pop('keys_x')
            # The robot has now moved, so we can send the ack
            buffer['response'] = 'move_x success'
    # og.log.info(f"act_x2: {'act_x' in local_vars}")
    if 'act_x' in local_vars:
        action[0] = local_vars['act_x']
    
    if 'act_y' in local_vars:
        forward_or_back, init_y, sy = local_vars['forward_or_back'], local_vars['init_y'], local_vars['sy']
        if forward_or_back * (robot.get_position()[0] - init_y) >= abs(sy):
            for key in local_vars['keys_y']:
                local_vars.pop(key)
            local_vars.pop('keys_y')
            buffer['response'] = 'move_y success'
    if 'act_y' in local_vars:
        action[1] = local_vars['act_y']
    
    if 'act_turn' in local_vars:
        if local_vars['continue_steps'] <= 0:
            for key in local_vars['keys_turn']:
                local_vars.pop(key)
            local_vars.pop('keys_turn')
            buffer['response'] = 'turn success'
        else:
            local_vars['continue_steps'] -= 1
    if 'act_turn' in local_vars:
        action[2] = local_vars['act_turn']
    
    local_vars['action'] = action
    return


if __name__ == "__main__":
    main()
