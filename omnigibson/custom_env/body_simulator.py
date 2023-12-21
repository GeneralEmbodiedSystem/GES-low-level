# -*- coding: UTF-8 -*-

# mount tmpfs:
# mkdir ~/tmpfs
# sudo mount -t tmpfs -o size=512M tmpfs ~/tmpfs/

"""
Example script demo'ing robot manipulation control with grasping.
"""
import multiprocessing as mp

import numpy as np
import yaml

import omnigibson as og
from omnigibson.utils.low_level_utils import convert_ndarray_in_dict_to_shape, omit_convert_dict
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from omnigibson.utils.ui_utils import choose_from_options, KeyboardRobotController
from omnigibson.utils.asset_utils import get_available_g_scenes, get_available_og_scenes
from omnigibson.robots import FetchAdvanced, TiagoAdvanced
from servers import PipeManager
from omnigibson.custom_env.skills.get_image import run_get_image
from omnigibson.custom_env.skills.move_forward import run_move_forward, execute_move_forward
from omnigibson.custom_env.skills.move_backward import run_move_backward, execute_move_backward
from omnigibson.custom_env.skills.move_left import run_move_left, execute_move_left
from omnigibson.custom_env.skills.move_right import run_move_right, execute_move_right
from omnigibson.custom_env.skills.turn_left import run_turn_left, execute_turn_left
from omnigibson.custom_env.skills.turn_right import run_turn_right, execute_turn_right
from omnigibson.custom_env.skills.brake import run_brake


from omnigibson.utils.ui_utils import create_module_logger
log = create_module_logger(module_name=__name__)

# write your skills and corresponding function here!!
skillset = {
    'get_image': (run_get_image, ),
    'move_forward': (run_move_forward, execute_move_forward),
    'move_backward': (run_move_backward, execute_move_backward),
    'move_left': (run_move_left, execute_move_left),
    'move_right': (run_move_right, execute_move_right),
    'turn_left': (run_turn_left, execute_turn_left),
    'turn_right': (run_turn_right, execute_turn_right),
    'brake': (run_brake, )
}

# Don't use GPU dynamics and Use flatcache for performance boost
gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True

# Read the configuration file
SERVER_CONFIG_PATH = './servers/config.yaml'
with open(SERVER_CONFIG_PATH, 'r') as f:
    server_config = yaml.load(f, Loader=yaml.FullLoader)

TMP_PATH = server_config['tmp_path']
SHM_NAME_INTER = server_config['shm_ser_sim']['name']
SHM_SIZE_INTER = server_config['shm_ser_sim']['size']

SIM_CONFIG_PATH = './omnigibson/custom_env/body_simulator_cfg.yaml'
with open(SIM_CONFIG_PATH, 'r') as f:
    sim_config = yaml.load(f, Loader=yaml.FullLoader)


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

    # Create environment configuration to use
    scene_cfg = sim_config["scene_cfg"]
    robot0_cfg = sim_config["robot0_cfg"]

    # type of grasping
    grasping_mode = robot0_cfg['grasping_mode']

    # Define objects to load
    objects_cfg = sim_config["objects_cfg"]

    # Compile config
    cfg = dict(scene=scene_cfg, robots=[robot0_cfg], objects=objects_cfg)

    # Create the environment
    env = og.Environment(configs=cfg, action_timestep=1/60., physics_timestep=1/60.)

    # Reset the robot
    robot = env.robots[0]

    # TODO: remove hard code
    robot.set_position([0, -0.5, 0])
    robot.reset()
    # robot.keep_still()

    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = 720
            sensor.image_width = 720
    
    # TODO: remove hard code
    # Update the simulator's viewer camera's pose so it points towards the robot
    og.sim.viewer_camera.set_position_orientation(
        position=np.array([1.20465,  1.51194,  2.14789]),
        orientation=np.array([0.144,  0.424,  0.846, 0.28757]),
    )

    # Create teleop controller
    # action_generator = KeyboardRobotController(robot=robot)  
    # If you want to use keyboard teleop, please refer to the example in omnigibson/examples/custom/grasp-interactive_scene.py

    # Create a dequeue to store the active actions
    # These actions will be executed every time step
    active_actions = {
        # The action related to base moving, E.g.,
        # "move_action": {
        #     "func": execute_example_skill,
        #     "args": {
        #         "arg_1": 0.1 
        #     },
        #     # Max timesteps to execute this action (if necessary)
        #     "timeout": 100
        # },
        # None means no action (still)
        "move_action": None,
        # TODO: implement the action related to arm moving
        "hand_action": None
    }

    # Other helpful user info
    print(f"Running demo with grasping mode {grasping_mode} and scene mode{scene_cfg['scene_model']}.")
    print("Use CTRL+C to quit")

    # Loop control until user quits
    # short_exec = True
    # max_steps = -1 if not short_exec else 600
    # step = 0
    # action_random_selection = False
    
    try:
        while True:
            action = np.zeros(robot.action_dim)
            execute_message(buffer, locals())

            """
            TiagoAdvance action idx (action_dim=19):
            0: base move (x axis), + means forward and - means back. 0.1 * 84 steps ~ 0.2m.  0.1 * 1 step ~ [0.00238, 0.00244]m
            1: base move (y axis), + means left and - means right. 0.1 * 50 steps ~ 0.1173m.  0.1 * 1 step ~ 0.002346 m
            2: turn base (+left/-right). 614 steps * 0.2 = 122.8 ~ 360 degree left;  61.4 ~ 180 degree left;
                so 90 degree left is 30.7 or 0.1 * 307 steps; or 0.6 * 51 steps (action[2]=0.6 is large enough!)
            3: turn head (+left/-right)
            4: head +up/-down
            [5, 11]: left arm. for example, the dim 5 means left hand front/back 
            [12, 18]: right arm.
            """
            generate_action(buffer, locals())
            env.step(action)
    except KeyboardInterrupt:
        # Always shut down the environment cleanly at the end
        env.close()
        P.terminate()


def execute_message(buffer: dict, local_vars: dict):
    '''this function only set the low level to a given state. the actual movement(generate action at each time) is done in generate_action
    @param message(tuple): (CMD, *args)
        (get_image, ...)
        (move_forward, velocity)
        (move_backward, velocity)
        ...
    '''
    if buffer['message'] is None:
        return
    message = buffer['message']
    buffer['message'] = None
    
    og.log.info(f"message: {message}")
    # `skillset[message[0]][0]` is like run_xxx. 
    # It only set the action to be taken
    skillset[message[0]][0](buffer, local_vars, *message[1:])


def generate_action(buffer: dict, local_vars: dict):
    """
    Generate action for the robot according to `active_actions`.
    This function is called at each time step.
    """
    env, robot = local_vars['env'], local_vars['robot']
    action = local_vars['action']

    # Like execute_xxx, it applies the action to the robot action vector
    skill_executed = local_vars['active_actions']['move_action']
    if skill_executed is not None:
        skill_execute_func = skillset[skill_executed['func']][1]
        args = skill_executed['args']
        action = skill_execute_func(buffer, local_vars, **args)
    
    local_vars['action'] = action
    return


if __name__ == "__main__":
    main()
