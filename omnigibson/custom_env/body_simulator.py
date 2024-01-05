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
from omnigibson.custom_env.utils import handle_collision
from omnigibson.custom_env.skills.get_image import emit_get_image
from omnigibson.custom_env.skills.move_forward import emit_move_forward, execute_move_forward
from omnigibson.custom_env.skills.move_backward import emit_move_backward, execute_move_backward
from omnigibson.custom_env.skills.move_left import emit_move_left, execute_move_left
from omnigibson.custom_env.skills.move_right import emit_move_right, execute_move_right
from omnigibson.custom_env.skills.turn_left import emit_turn_left, execute_turn_left
from omnigibson.custom_env.skills.turn_right import emit_turn_right, execute_turn_right
from omnigibson.custom_env.skills.move_gripper_to import emit_move_gripper_to, execute_move_gripper_to
from omnigibson.custom_env.skills.close_gripper import emit_close_gripper, execute_close_gripper
from omnigibson.custom_env.skills.open_gripper import emit_open_gripper, execute_open_gripper
from omnigibson.custom_env.skills.raise_gripper import emit_raise_gripper, execute_raise_gripper
from omnigibson.custom_env.skills.look_up import emit_look_up, execute_look_up
from omnigibson.custom_env.skills.look_forward import emit_look_forward, execute_look_forward
from omnigibson.custom_env.skills.look_down import emit_look_down, execute_look_down
from omnigibson.custom_env.configs import ENV_CONFIG_PATH

from omnigibson.robots import TiagoAdvanced

from omnigibson.utils.transform_utils import quat2euler

from omnigibson.custom_env.utils.camera_utils import execute_record

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
    'move_gripper_to': (emit_move_gripper_to, execute_move_gripper_to),
    'close_gripper': (emit_close_gripper, execute_close_gripper),
    'open_gripper': (emit_open_gripper, execute_open_gripper),
    'raise_gripper': (emit_raise_gripper, execute_raise_gripper),
    'look_up': (emit_look_up, execute_look_up),
    'look_forward': (emit_look_forward, execute_look_forward),
    'look_down': (emit_look_down, execute_look_down),
}

# Don't use GPU dynamics and Use flatcache for performance boost
gm.ENABLE_OBJECT_STATES = True  # Make sure object states are enabled
gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True

# Read the configuration file
SERVER_CONFIG_PATH = './servers/config.yaml'
with open(SERVER_CONFIG_PATH, 'r') as f:
    server_config = yaml.load(f, Loader=yaml.FullLoader)

TMP_PATH = server_config['tmp_path']
SHM_NAME_INTER = server_config['shm_ser_sim']['name']
SHM_SIZE_INTER = server_config['shm_ser_sim']['size']

def lift_trunk_body(robot: TiagoAdvanced): # TiagoAdvanced
    robot.set_joint_positions(
        np.array([1.]),
        indices=robot.trunk_control_idx,
        # drive=True,  # if drive=True, the robot will gradually move to the target position. else it will be set to the target position immediately
    )


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_file", "-c", type=str, default="table_food_trash_can_env.yaml")
    parser.add_argument("--record", "-r", type=int, default=None)
    args = parser.parse_args()
    
    return args


def main(random_selection=False, headless=False, short_exec=False, args=get_args()):
    """
    Robot grasping mode demo with selection
    Queries the user to select a type of grasping mode
    """
    # environment config, including scene, robot, and objects
    sim_config_path = ENV_CONFIG_PATH + "/" + args.config_file
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
    robot: TiagoAdvanced = env.robots[0]

    robot.set_position(sim_config["other_init_cfg"]["robot_position"])  # robot0_cfg["position"] will be modified! DON'T USE IT!
    robot.set_orientation(sim_config["other_init_cfg"]["robot_orientation"]) 
    robot.reset()
    
    robot.lift_trunk_body()
    og.sim.step()
    # robot.keep_still()

    # Lidar is DISABLED temporarily
    # because the orientation of lidar's axis
    # is absolute, not aware of robot's orientation
    # Init Lidar sensors
    # robot.init_scan_sensor()
    
    # limit the max joint velocities of the robot
    robot.max_joint_velocities[robot.ik_control_idx] *= 1
    robot.max_joint_efforts[robot.ik_control_idx] *= 1
    
    # Init record frames cache
    if args.record is not None:
        print(f'record {args.record} frames!')
        viewer_record_frames = []
        robot_camera_record_frames = []

    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = sim_config["other_init_cfg"]["camera_resolution"]["image_height"]
            sensor.image_width = sim_config["other_init_cfg"]["camera_resolution"]["image_width"]
    
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
        # Exception related to base movement
        "move_exception": None,
        # Actions related to arm movement
        "hand_action": None,
        # Actions related to head movement
        "head_action": None,
        # robot_states
        'robot_states': {
            'gripper_state': 'open', # 'open' or 'close'
        }
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
    # ignore_objs = tuple(
    #     env.scene.object_registry("category", "floors", []),
    #     # env.scene.object_registry("category", "box_of_yogurt", []),
    # )
    
    # Utility variables for collision detection
    collision_vars = {
        'last_position': None,
        # 'init_orientation_z': None,
    }
    
    # set GLOBAL_STATE buffer (dict)
    global_states = dict()
    global_states_keys = ['env', 'robot', 'active_actions', 'collision_vars']
    
    for global_states_key in global_states_keys:
        global_states[global_states_key] = eval(global_states_key)
    
    try:
        steps = 0
        while True:
            # If a collision happens, brake the robot
            handle_collision(global_states)
            
            # Prepare the action vector
            global_states['action'] = np.zeros(robot.action_dim)
            if robot.ready:
                emit_message(pipe, global_states)  # call emit_xxx and prepare for generate_action()

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
            update_env(robot, global_states)
            steps += 1
            
            # Record video frames
            if args.record is not None:
                if steps < args.record:
                    execute_record(og.sim.viewer_camera, viewer_record_frames, './viewer.mp4')
                    execute_record(robot.vision_sensor, robot_camera_record_frames, './robot_camera.mp4')
                elif steps == args.record:
                    execute_record(og.sim.viewer_camera, viewer_record_frames, './viewer.mp4', end=True)
                    execute_record(robot.vision_sensor, robot_camera_record_frames, './robot_camera.mp4', end=True)
    except KeyboardInterrupt:
        # Always shut down the environment cleanly at the end
        env.close()


def emit_message(pipe: PipeManager, global_states: dict):
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
    move_exception = global_states['active_actions']['move_exception']

    # Like execute_xxx, it applies the action to the robot action vector
    skill_executed_list = ['move_action', 'hand_action', 'head_action']
    
    # Recover exception
    if move_exception is not None:
        if move_exception.cur_recover_step == 0:
            # TODO: put this in specific skill
            # Report the exception to the server
            pipe.write(
                data=str(move_exception),
                role="server"
            )
        move_exception.cur_recover_step += 1
        # If the robot collides with the environment, get an recovery action
        action = move_exception.get_recovery_action(action, global_states)
        if move_exception.cur_recover_step >= move_exception.recover_steps:
            # If the robot has recovered from the exception, make it ready
            global_states['active_actions']['move_exception'] = None
            robot.ready = True
    else:
        # Execute the skills
        for skill_executed in skill_executed_list:
            skill_executed = global_states['active_actions'].get(skill_executed, None)
            if skill_executed is not None:
                skill_execute_func = skillset[skill_executed['func']][1]
                args = skill_executed['args']
                action = skill_execute_func(pipe, global_states, **args)
    
    global_states['action'] = action


def update_env(robot: TiagoAdvanced, global_states: dict):
    """
    Apply the robot action and step the simulation.

    In this function, we use the physx interface (og.sim.step()) 
    to step the action instead of using the env.step() function.
    This is due to unknown compatibility issues.
    """
    # prior params
    left_or_right = 'left'
    
    # Whether to use drive mode
    use_drive = True
    # If we have move_left or move_right,
    # set use_drive to False to avoid bugs of air walls
    if global_states['active_actions']['move_action'] is not None:
        if global_states['active_actions']['move_action']['func'] in \
            ['move_left', 'move_right']:
            use_drive = False

    # extract vars
    action = global_states['action']
    
    # init tgt positions and velocities list
    tgt_joint_poses, tgt_joint_pos_idxes = [], []
    tgt_joint_vels, tgt_joint_vel_idxes = [], []
    
    # target position control
    # gripper movement control
    skill_executed_list = ['hand_action', 'head_action']
    for skill_executed in skill_executed_list:
        if skill_executed in global_states['active_actions']:
            physx_action = global_states['active_actions'][skill_executed]
            if isinstance(physx_action, dict):
                # print("physx_action", physx_action)
                if 'tgt_joint_pos' in physx_action:
                    for k, v in physx_action['tgt_joint_pos'].items():
                        tgt_joint_poses.append(v[0])
                        tgt_joint_pos_idxes.append(v[1])
    # gripper open or close state control
    if 'robot_states' in global_states['active_actions']:
        if 'gripper_state' in global_states['active_actions']['robot_states']:
            gripper_state = global_states['active_actions']['robot_states']['gripper_state']
            if gripper_state == 'open':
                tgt_joint_poses.append([0.045, 0.045])
            else:
                tgt_joint_poses.append([0.0, 0.0])
            tgt_joint_pos_idxes.append(robot.gripper_control_idx[left_or_right])
    # implement the position control
    if len(tgt_joint_poses) > 0:
        tgt_joint_pos = np.concatenate(tgt_joint_poses)
        tgt_joint_pos_idx = np.concatenate(tgt_joint_pos_idxes)
        robot.set_joint_positions(
            tgt_joint_pos, 
            indices=tgt_joint_pos_idx,
            drive=use_drive,
        )
    
    # target velocities control
    # robot movement control
    if action is not None:
        # base movement control
        tgt_joint_vels.append(action[:3])
        tgt_joint_vel_idxes.append(robot.base_control_idx)
    # implement the velocity control
    if len(tgt_joint_vels) > 0:
        tgt_joint_vel = np.concatenate(tgt_joint_vels)
        tgt_joint_vel_idx = np.concatenate(tgt_joint_vel_idxes)
        robot.set_joint_velocities(
            tgt_joint_vel, 
            indices=tgt_joint_vel_idx, 
            drive=use_drive,
        )
    
#     file_log(f'''robot orientation: {robot.get_orientation()}
# quat2euler: {quat2euler(robot.get_orientation())}
# ''')
    
    og.sim.step()

def file_log(log: str):
    with open('./tmp.log', 'w') as f:
        f.write(log)

if __name__ == "__main__":
    main()
