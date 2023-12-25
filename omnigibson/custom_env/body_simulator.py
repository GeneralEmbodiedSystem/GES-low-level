# -*- coding: UTF-8 -*-
"""
Example script demo'ing robot manipulation control with grasping.
"""
import numpy as np
import yaml
import argparse

import omnigibson as og
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from servers import PipeManager
from omnigibson.custom_env.exceptions import MoveException
from omnigibson.custom_env.utils import handle_collision
from omnigibson.custom_env.skills.get_image import emit_get_image
from omnigibson.custom_env.skills.move_forward import emit_move_forward, execute_move_forward
from omnigibson.custom_env.skills.move_backward import emit_move_backward, execute_move_backward
from omnigibson.custom_env.skills.move_left import emit_move_left, execute_move_left
from omnigibson.custom_env.skills.move_right import emit_move_right, execute_move_right
from omnigibson.custom_env.skills.turn_left import emit_turn_left, execute_turn_left
from omnigibson.custom_env.skills.turn_right import emit_turn_right, execute_turn_right
from omnigibson.custom_env.skills.brake import emit_brake
from omnigibson.custom_env.skills.get_velocity import emit_get_velocity
from omnigibson.custom_env.skills.reach import emit_reach, execute_reach
from omnigibson.custom_env.skills.grasp import emit_grasp, execute_grasp
from omnigibson.custom_env.skills.release import emit_release, execute_release


from omnigibson.utils.ui_utils import create_module_logger
log = create_module_logger(module_name=__name__)

# write your skills and corresponding function here!!
skillset = {
    'get_image': (emit_get_image, ),
    'move_forward': (emit_move_forward, execute_move_forward),
    'move_backward': (emit_move_backward, execute_move_backward),
    'move_left': (emit_move_left, execute_move_left),
    'move_right': (emit_move_right, execute_move_right),
    'turn_left': (emit_turn_left, execute_turn_left),
    'turn_right': (emit_turn_right, execute_turn_right),
    'brake': (emit_brake, ),
    'get_velocity': (emit_get_velocity, ),
    'reach': (emit_reach, execute_reach),
    'grasp': (emit_grasp, execute_grasp),
    'release': (emit_release, execute_release),
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

def lift_trunk_body(robot): # TiagoAdvanced
    robot.set_joint_positions(
        np.array([1.]),
        indices=robot.trunk_control_idx,
        # drive=True,  # if drive=True, the robot will gradually move to the target position. else it will be set to the target position immediately
    )


def main(random_selection=False, headless=False, short_exec=False):
    """
    Robot grasping mode demo with selection
    Queries the user to select a type of grasping mode
    """
    # environment config, including scene, robot, and objects
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_file", type=str, default="table_food_trash_can_env.yaml")
    args = parser.parse_args()
    sim_config_path = './omnigibson/custom_env/configs/' + args.config_file
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.load(f, Loader=yaml.FullLoader)
    
    # Create pipe for communication with server process
    # This is for internal communication between the simulator process 
    # and the server process. Here, the 'client' is the server process,
    # which forwards the remote client's request to the simulator.
    # And the 'server' is the simulator process, which handles the request
    # and returns the response to the server process.
    pipe = PipeManager(
        TMP_PATH, SHM_NAME_INTER, 
        SHM_SIZE_INTER, create=False
    )

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
    # robot.set_position(sim_config["other_init_cfg"]["robot_position"])  # robot0_cfg["position"]
    # robot.reset()
    
    lift_trunk_body(robot)
    og.sim.step()
    # robot.keep_still()

    # Init Lidar sensors
    robot.init_scan_sensor()

    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = sim_config["other_init_cfg"]["camera_resolution"]["image_height"]
            sensor.image_width = sim_config["other_init_cfg"]["camera_resolution"]["image_width"]
    
    # TODO: remove hard code
    # Update the simulator's viewer camera's pose so it points towards the robot
    og.sim.viewer_camera.set_position_orientation(
        position=np.array(sim_config["viewer_camera_cfg"]["position"]),
        orientation=np.array(sim_config["viewer_camera_cfg"]["orientation"]),
    )
    og.sim.step()

    # Create teleop controller
    # action_generator = KeyboardRobotController(robot=robot)  
    # If you want to use keyboard teleop, please refer to the example in omnigibson/examples/custom/grasp-interactive_scene.py

    # Create a dequeue to store the active actions
    # These actions will be executed every time step
    active_actions = {
        # The action related to base moving, E.g.,
        # "move_action": {
        #     "func": "execute_example_skill",
        #     "args": {
        #         "arg_1": 0.1 
        #     },
        #     # Max timesteps to execute this action (if necessary)
        #     "timeout": 100
        # },
        # None means no action (still)
        # Actions related to base movement
        "move_action": None,
        # Actions related to arm movement
        "hand_action": None,
        # TODO: Actions related to head movement
    }

    # Other helpful user info
    print(f"Running demo with grasping mode {grasping_mode} and scene mode{scene_cfg.get('scene_model', None)}.")
    print("Use CTRL+C to quit")

    # Loop control until user quits
    # short_exec = True
    # max_steps = -1 if not short_exec else 600
    # step = 0
    # action_random_selection = False

    # Ignore the floors when checking collision
    ignore_objs = tuple(env.scene.object_registry("category", "floors", []))
    
    # set GLOBAL_STATE buffer (dict)
    global_states = dict()
    global_states_keys = ['env', 'robot', 'active_actions']
    for global_states_key in global_states_keys:
        global_states[global_states_key] = eval(global_states_key)
    
    try:
        while True:
            # If a collision happens, brake the robot
            handle_collision(global_states, ignore_objs)
            # Prepare the action vector
            if global_states['active_actions']['hand_action'] is None:
                global_states['action'] = np.zeros(robot.action_dim)
            else:
                global_states['action'] = None
            if robot.ready:
                execute_message(pipe, global_states)  # calll emit_xxx and prepare for generate_action()

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
            generate_action(pipe, global_states)  # call execute_xxx and set action
            action = global_states['action']
            
            if active_actions['hand_action'] is not None:
                # print("ji ni tai mei ", action)
                pass
            if action is None:
                # Use the physx interface to step the action
                # print("step arm action by physx")
                step_arm_action_by_physx(robot, global_states)
            else:
                env.step(action)
    except KeyboardInterrupt:
        # Always shut down the environment cleanly at the end
        env.close()


def execute_message(pipe: PipeManager, global_states: dict):
    '''this function only set the low level to a given state. the actual movement(generate action at each time) is done in generate_action
    @param message(tuple): (CMD, *args)
        (get_image, ...)
        (move_forward, velocity)
        (move_backward, velocity)
        ...
    '''
    message = pipe.read(role='server')
    if message is None:
        return
    
    og.log.info(f"message: {message}")
    # `skillset[message[0]][0]` is like run_xxx. 
    # It only set the action to be taken
    skillset[message[0]][0](pipe, global_states, *message[1:])


def generate_action(pipe: PipeManager, global_states: dict):
    """
    Generate action for the robot according to `active_actions`.
    This function is called at each time step.
    """
    env, robot = global_states['env'], global_states['robot']
    action = global_states['action']

    # Like execute_xxx, it applies the action to the robot action vector
    skill_executed_list = [global_states['active_actions']['move_action'],
                            global_states['active_actions']['hand_action']]
    for skill_executed in skill_executed_list:
        if isinstance(skill_executed, MoveException) and not skill_executed.recovered:
            # If the robot collides with the environment, get an recovery action
            skill_executed = skill_executed.get_recovery_action()
            global_states['active_actions']['move_action'].recovered_steps -= 1
            if global_states['active_actions']['move_action'].recovered_steps <= 0:
                # If the robot has recovered from the exception, make it ready
                global_states['active_actions']['move_action'].recovered = True
                robot.ready = True
        if skill_executed is not None and not isinstance(skill_executed, MoveException):
            skill_execute_func = skillset[skill_executed['func']][1]
            args = skill_executed['args']
            action = skill_execute_func(pipe, global_states, **args)
    
    global_states['action'] = action
    # if 'hand_action' in global_states['active_actions']:
    #     print("hand_action", global_states['active_actions']['hand_action'])
    #     if "move_action" in global_states['active_actions']:
    #         print("move_action", global_states['active_actions']['move_action'])
    #     print("action", action)
    if global_states['active_actions']['hand_action'] is not None:
        # print("WO G ", action)
        pass
    return


def step_arm_action_by_physx(robot, global_states: dict):
    """
    In this function, we use the physx interface (og.sim.step()) 
    to step the arm action instead of using the env.step() function.
    This is due to unknown compatibility issues.
    """
    physx_action = global_states['active_actions']['hand_action']
    tgt_joint_poses, tgt_control_idxes = [], []
    # print("physx_action", physx_action)
    for k, v in physx_action['tgt_joint_pos'].items():
        tgt_joint_poses.append(v[0])
        tgt_control_idxes.append(v[1])
    if len(tgt_joint_poses) > 0:
        print(f'drive to tgt joint positions.')
        tgt_joint_pos = np.concatenate(tgt_joint_poses)
        tgt_control_idx = np.concatenate(tgt_control_idxes)
        robot.set_joint_positions(
            tgt_joint_pos, 
            indices=tgt_control_idx,
            drive=True,
        )
    
    # simulator step
    og.sim.step()


if __name__ == "__main__":
    main()
