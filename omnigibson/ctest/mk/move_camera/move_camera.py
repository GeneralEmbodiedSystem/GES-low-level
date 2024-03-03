#!/usr/bin/env python
# mk origin ik(to avoid forget)

import os
import time

import matplotlib.pyplot as plt
import numpy as np

import omnigibson as og
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from omnigibson.robots.turtlebot import Turtlebot
from omnigibson.robots.tiago import Tiago
from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.robots import FetchAdvanced
from omnigibson.scenes.interactive_traversable_scene import InteractiveTraversableScene
from omnigibson.utils.asset_utils import get_og_assets_version
from omnigibson.utils.ui_utils import choose_from_options, KeyboardRobotController
from omnigibson.utils.control_utils import IKSolver
from omnigibson.objects import PrimitiveObject

import carb
import omni

from utils.prim_utils import get_pos_by_prim_path, get_pos_by_prim_path_physxApi
from utils.simulator_utils import custom_step
from utils.skill_utils import lift_trunk_body, execute_grasp, execute_ik, execute_look_at, \
    execute_record

# Params to be set as needed.
SCENES = ["Beechwood_0_int"]
OUTPUT_DIR = os.path.join(os.path.expanduser("~"), "Desktop")
NUM_STEPS = 2000
gm.DEFAULT_VIEWER_WIDTH = 720
gm.DEFAULT_VIEWER_HEIGHT = 720  # high-res


def benchmark_scene(scene_name, optimized=False, import_robot=True, random_selection=False):
    assets_version = get_og_assets_version()
    print("assets_version", assets_version)
    scene = InteractiveTraversableScene(
        scene_model=scene_name,
        load_object_categories=["floors", "walls", "ceilings"], # use complex scenes will block the arm of the robot
    )
    start = time.time()
    og.sim.import_scene(scene)
    print(f"import scene consume time: {time.time() - start:.4f}s")

    # og.sim.viewer_camera.set_position_orientation(
    #     position=np.array([-13.27634359, -12.70026879,  -2.68970369]),
    #     orientation=np.array([0.70597155, 0.10758371, 0.10545935, 0.69203196]),
    # )
    
    # back view
    # og.sim.viewer_camera.set_position_orientation(
    #     position=np.array([-1.41302,  1.50739,  2.09367]),
    #     orientation=np.array([-0.23898,  0.48475, 0.75464, -0.37205]), # here is xyzw, in isaac sim wxyz
    # )
    
    # front view
    # og.sim.viewer_camera.set_position_orientation(
    #     position=np.array([1.83795, 0.01503, 0.69411]),
    #     orientation=np.array([0.50083, 0.51367, 0.49881, 0.48632]), # here is xyzw, in isaac sim wxyz
    # )

    # side view
    # og.sim.viewer_camera.set_position_orientation(
    #     position=np.array([1.14539, 1.00671, 0.68562]),
    #     orientation=np.array([0.21118, 0.65232, 0.69255, 0.22419]), # here is xyzw, in isaac sim wxyz
    # )

    # full side view simple scene
    og.sim.viewer_camera.set_position_orientation(
        position=np.array([1.32583, 1.49008, 1.04465]),
        orientation=np.array([0.12579, 0.59505, 0.77662, 0.16416]), # here is xyzw, in isaac sim wxyz
    )
    
    # full side view complex scene
    # og.sim.viewer_camera.set_position_orientation(
    #     position=np.array([1.3, -1.0, 1.4]),
    #     orientation=np.array([0.57336, 0.07555, 0.10124, 0.80951]), # here is xyzw, in isaac sim wxyz
    # )

    # # Hide main viewport
    # og.sim.viewer_visibility = False

    robot_choice = 'TiagoAdvanced' # ['TiagoAdvanced', 'FetchAdvanced']
    if robot_choice == 'TiagoAdvanced':
        robot = TiagoAdvanced(
            prim_path="/World/robot",
            name="robot",
            # fixed_base=True,
            controller_config={
                "camera": {
                    "name": "JointController",
                    "motor_type": "position",
                },
                "arm_left": {
                    "name": "JointController",
                    "motor_type": "position",
                },
                "arm_right": {
                    "name": "JointController",
                    "motor_type": "position",
                }
            } # very useful
        )
    # elif robot_choice == 'FetchAdvanced':
    #     robot = FetchAdvanced(
    #         prim_path="/World/robot",
    #         name="robot",
    #         # fixed_base=True,
    #         controller_config={
    #             "arm_0": {
    #                 "name": "JointController",
    #                 "motor_type": "position",
    #             }
    #         }
    #     )
    og.sim.import_object(robot)
    og.sim.step()
    og.log.info(f"clipping_range: {robot.clipping_range}")
    og.log.info(f"focal_length: {robot.focal_length}")
    if scene_name == 'Beechwood_0_int':
        robot.set_position_orientation(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0, 0, 0, 1.0])
        )
        pass
    elif scene_name == "restaurant_hotel":
        robot.set_position_orientation(
            position=[-12.2627573, -10.60511875, -3.4790891],
            orientation=[0, 0, 0, 1.0]
        )
    
    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = 1080
            sensor.image_width = 1920
    
    # TODO: still buggy
    # action_generator = KeyboardRobotController(robot=robot)
    # # Print out relevant keyboard info if using keyboard teleop
    # action_generator.print_keyboard_teleop_info()
    
    og.sim.play()
    og.sim.step()
    
    # keep robot 
    ''' create ik solver '''
    robot.keep_still()
    control_idx = np.concatenate([robot.trunk_control_idx, robot.arm_control_idx[robot.default_arm]])
    ik_solver = IKSolver(
        robot_description_path=robot.robot_arm_descriptor_yamls[robot.default_arm],
        robot_urdf_path=robot.urdf_path,
        default_joint_pos=robot.get_joint_positions()[control_idx],
        eef_name=robot.eef_link_names[robot.default_arm],
    )
    
    # ''' adjust camera orientation '''
    # robot.set_joint_positions(
    #     positions=np.array([0., 1.]),
    #     indices=robot.camera_control_idx,
    #     drive=True,
    # )
    # og.sim.step()

    ''' create a goal '''
    # Create a visual marker to be moved by the user, representing desired end-effector position
    marker = PrimitiveObject(
        prim_path=f"/World/marker",
        name="marker",
        primitive_type="Sphere",
        radius=0.03,
        visual_only=True,
        rgba=[1.0, 0, 0, 1.0],
        # position=np.array([0.25, -0.21, 0.22]),
    )
    og.sim.import_object(marker)
    
    # Get initial EE position and set marker to that location
    # command = robot.get_eef_position()
    command = np.array([0.34555, 0.25, 0.80099]) # simple scene
    # command = np.array([1.33247, 0.25, 0.73184]) # complex scene
    marker.set_position(command)
    og.sim.step()

    # Setup callbacks for grabbing keyboard inputs from omni
    exit_now = False

    def keyboard_event_handler(event, *args, **kwargs):
        nonlocal command, exit_now
        # Check if we've received a key press or repeat
        if event.type == carb.input.KeyboardEventType.KEY_PRESS \
                or event.type == carb.input.KeyboardEventType.KEY_REPEAT:
            if event.input == carb.input.KeyboardInput.ENTER:
                # Execute the command
                # print(f"execute_ik pos: {command}")
                # print(f'command: {command}')
                execute_ik(ik_solver, robot, command, buffer)
            elif event.input == carb.input.KeyboardInput.ESCAPE:
                # Quit
                og.log.info("Quit.")
                exit_now = True
            else:
                # We see if we received a valid delta command, and if so, we update our command and visualized
                # marker position
                delta_cmd = input_to_xyz_delta_command(inp=event.input)
                if delta_cmd is not None:
                    command = command + delta_cmd
                    marker.set_position(command)
                    # print(f"marker pos: {command}")
                    og.sim.step()

        # Callback must return True if valid
        return True

    # Hook up the callback function with omni's user interface
    appwindow = omni.appwindow.get_default_app_window()
    input_interface = carb.input.acquire_input_interface()
    keyboard = appwindow.get_keyboard()
    sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard, keyboard_event_handler)

    # Print out helpful information to the user
    print_message()
    
    max_steps = -1
    # max_steps = 400
    step = 0
    buffer = dict(); buffer['tgt_joint_pos'] = dict()
    while step != max_steps:
        # print("env.objects_config", env.objects_config)  # no position here
        # apple_pos = get_pos_by_prim_path("/World/delicious_apple/base_link") - position_mapping['delicious_apple']  # notice the pos!
        # apple_pos = get_pos_by_prim_path_physxApi("/World/delicious_apple/base_link") # this is world position
        # box_pos = get_pos_by_prim_path_physxApi("/World/box/base_link") # this is world position
        # robot_pos = get_pos_by_prim_path_physxApi("/World/robot/base_link") # this is world position
        marker_pos = get_pos_by_prim_path_physxApi("/World/marker/base_link") # this is world position
        # print(f'robot_pos: {robot_pos}')
        tgt_pos = marker_pos
        print("tgt pos", tgt_pos)
        
        # grasp(robot)
        
        if step % 3 == 0:
            # execute_ik(ik_solver, robot, tgt_pos, buffer, quat=np.array([0, 0, 0, 1]),
            #            weight_pos=100000.0, weight_quat=0.001)
            execute_ik(ik_solver, robot, tgt_pos, buffer)
        # execute_grasp(robot, buffer, 'left')
        # og.log.info(f'mk action_idx: {action_idx}, action_val: {action_val}')
        # print(f'joint pos: {robot.get_joint_positions()[control_idx]}')
        
        # teleop_action = action_generator.get_teleop_action()
        # teleop_action[2:] = 0 # no press key output 2 0??
        # teleop_idx, teleop_val = extract_valid_pos_and_val(teleop_action)
        # og.log.info(f'mk teleop_idx: {teleop_idx}, teleop_val: {teleop_val}')
        
        # # action[action_idx] = action_val
        # action[teleop_idx] = teleop_val
        
        # og.log.info(f'mk action: {action}')
        # env.step(action)
        # for _ in range(10):
        execute_look_at(robot, tgt_pos, buffer)
        
        custom_step(robot, buffer)
        step += 1
    
    og.app.close()

def input_to_xyz_delta_command(inp, delta=0.01):
    mapping = {
        carb.input.KeyboardInput.W: np.array([delta, 0, 0]),
        carb.input.KeyboardInput.S: np.array([-delta, 0, 0]),
        carb.input.KeyboardInput.DOWN: np.array([0, 0, -delta]),
        carb.input.KeyboardInput.UP: np.array([0, 0, delta]),
        carb.input.KeyboardInput.A: np.array([0, delta, 0]),
        carb.input.KeyboardInput.D: np.array([0, -delta, 0]),
    }

    return mapping.get(inp)

def print_message():
    print("*" * 80)
    print("Move the marker to a desired position to query IK and press ENTER")
    print("W/S: move marker further away or closer to the robot")
    print("A/D: move marker to the left or the right of the robot")
    print("T/G: move marker up and down")
    print("ESC: quit")

def main():
    for scene in SCENES:
        benchmark_scene(scene, optimized=True, import_robot=True)
        # og.sim.stop()
        # benchmark_scene(scene, optimized=True, import_robot=False)

    og.shutdown()


if __name__ == "__main__":
    main()
