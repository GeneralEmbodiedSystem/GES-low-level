from typing import Union, Optional, Dict, List

import sys
sys.path.append('..')

import os
import time

import matplotlib.pyplot as plt
import numpy as np
import multiprocessing as mp

from utils.common import convert_ndarray_in_dict_to_shape

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

import pprint
pp = pprint.PrettyPrinter()

# Params to be set as needed.
SCENES = ["Beechwood_0_int"]
OUTPUT_DIR = os.path.join(os.path.expanduser("~"), "Desktop")
NUM_STEPS = 2000
gm.DEFAULT_VIEWER_WIDTH = 1920
gm.DEFAULT_VIEWER_HEIGHT = 1080  # high-res

# scenes = ["Beechwood_0_int",
#           "Beechwood_1_int",
#           "Benevolence_0_int",
#           "Benevolence_1_int",
#           "Benevolence_2_int",
#           "Ihlen_0_int",
#           "Ihlen_1_int",
#           "Merom_0_int",
#           "Merom_1_int",
#           "Pomaria_0_int",
#           "Pomaria_1_int",
#           "Pomaria_2_int",
#           "Rs_int",
#           "Wainscott_0_int",
#           "Wainscott_1_int"]

manager = mp.Manager()

def server_process(*args, **kwargs):
    og.log.info(f"args: {args}")
    cache, = args
    while True:
        # server socket: receive message from llm
        # ...
        message = ('get_image', ) # e.g. get image of the camera
        # message = ('movex', 1) # e.g. move forward 1 meter
        # pass message to Isaac Sim
        cache['message'] = message

def benchmark_scene(scene_name, optimized=False, import_robot=True, random_selection=False):
    # start server process
    global manager
    cache = manager.dict()
    P = mp.Process(target=server_process, args=(cache,))
    P.start()
    
    # get OmniGibson assets version
    assets_version = get_og_assets_version()
    print("assets_version", assets_version)
    
    # add scene
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
        position=np.array([1.40783, 1.2968, 0.50722]),
        orientation=np.array([0.26595, 0.75386, 0.56658, 0.19987]), # here is xyzw, in isaac sim wxyz
    )
    
    # full side view complex scene
    # og.sim.viewer_camera.set_position_orientation(
    #     position=np.array([1.3, -1.0, 1.4]),
    #     orientation=np.array([0.57336, 0.07555, 0.10124, 0.80951]), # here is xyzw, in isaac sim wxyz
    # )

    # # Hide main viewport
    # og.sim.viewer_visibility = False

    if import_robot:
        robot_choice = 'TiagoAdvanced' # ['TiagoAdvanced', 'FetchAdvanced']
        if robot_choice == 'TiagoAdvanced':
            robot = TiagoAdvanced(
                prim_path="/World/robot",
                name="robot",
                # fixed_base=True,
                # controller_config={
                #     "base": {
                #         "name": "DifferentialDriveController",
                #         "motor_type": "position",
                #     },
                #     "camera": {
                #         "name": "JointController",
                #         "motor_type": "position",
                #     },
                #     "arm_left": {
                #         "name": "JointController",
                #         "motor_type": "position",
                #     },
                #     "arm_right": {
                #         "name": "JointController",
                #         "motor_type": "position",
                #     }
                # } # no use
            )
        elif robot_choice == 'FetchAdvanced':
            robot = FetchAdvanced(
                prim_path="/World/robot",
                name="robot",
                # fixed_base=True,
                controller_config={
                    "arm_0": {
                        "name": "JointController",
                        "motor_type": "position",
                    }
                }
            )
        og.sim.import_object(robot)
        og.sim.step()
        og.log.info(f"clipping_range: {robot.clipping_range}")
        og.log.info(f"focal_length: {robot.focal_length}")
        if scene_name == 'Beechwood_0_int':
            # robot.set_position_orientation(
            #     position=np.array([0.8, 0.0, 0.0]),
            #     orientation=np.array([0, 0, 0, 1.0])
            # )
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
    if import_robot:
        ''' create ik solver '''
        robot.keep_still()
        control_idx = np.concatenate([robot.trunk_control_idx, robot.arm_control_idx[robot.default_arm]])
        ik_solver = IKSolver(
            robot_description_path=robot.robot_arm_descriptor_yamls[robot.default_arm],
            robot_urdf_path=robot.urdf_path,
            default_joint_pos=robot.get_joint_positions()[control_idx],
            eef_name=robot.eef_link_names[robot.default_arm],
        )
        # Define a helper function for executing specific end-effector commands using the ik solver
        def execute_ik(pos, quat=None, max_iter=100):
            # og.log.info("Querying joint configuration to current marker position")
            # Grab the joint positions in order to reach the desired pose target
            joint_pos = ik_solver.solve(
                target_pos=pos,
                target_quat=quat,
                max_iterations=max_iter,
            )
            if joint_pos is not None:
                # og.log.info("Solution found. Setting new arm configuration.")
                pass
            else:
                # og.log.info("EE position not reachable.")
                pass
            # camera adjust to the goal
            camera_joint_pos, camera_idx = robot.look_at(goal_pos=pos)
            # set joint position
            final_joint_pos = np.concatenate([joint_pos, camera_joint_pos])
            final_joint_idx = np.concatenate([control_idx, camera_idx])
            robot.set_joint_positions(final_joint_pos, indices=final_joint_idx, drive=True)
            # simulation app step
            og.sim.step()
        
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
        # marker_command = robot.get_eef_position()
        marker_command = np.array([0.34555, 0.25, 0.80099]) # simple scene
        # marker_command = np.array([1.33247, 0.25, 0.73184]) # complex scene
        marker.set_position(marker_command)
        og.sim.step()

        # Setup callbacks for grabbing keyboard inputs from omni
        exit_now = False

        def keyboard_event_handler(event, *args, **kwargs):
            nonlocal marker_command, exit_now
            # Check if we've received a key press or repeat
            if event.type == carb.input.KeyboardEventType.KEY_PRESS \
                    or event.type == carb.input.KeyboardEventType.KEY_REPEAT:
                if event.input == carb.input.KeyboardInput.ENTER:
                    # Execute the marker_command
                    # print(f"execute_ik pos: {marker_command}")
                    execute_ik(pos=marker_command)
                elif event.input == carb.input.KeyboardInput.ESCAPE:
                    # Quit
                    og.log.info("Quit.")
                    exit_now = True
                else:
                    # We see if we received a valid delta marker_command, and if so, we update our marker_command and visualized
                    # marker position
                    marker_delta, move_delta = keyboardInput_to_delta(inp=event.input)
                    if marker_delta is not None:
                        marker_command = marker_command + marker_delta
                        marker.set_position(marker_command)
                        # print(f"marker pos: {marker_command}")
                        og.sim.step()
                    # if move_delta is not None:
                        

            # Callback must return True if valid
            return True

        # Hook up the callback function with omni's user interface
        appwindow = omni.appwindow.get_default_app_window()
        input_interface = carb.input.acquire_input_interface()
        keyboard = appwindow.get_keyboard()
        sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard, keyboard_event_handler)

        # Print out helpful information to the user
        print_message()
    
    # pp.pprint(convert_ndarray_in_dict_to_shape(robot._default_controller_config))
    # print(robot.base_control_idx) # x, y, rz
    # print(robot.base_idx)
    
    while not exit_now:
        # execute_ik(pos=marker_command)
        execute_message(cache, locals())
        og.sim.step()
    
    og.app.close()
    # P.join(); P.close()
    P.terminate()
    return

def execute_message(cache: dict, local_vars: dict):
    '''
    @param message(tuple): (CMD, *args)
        (get_image, )
        (movex, distance)
        (movey, distance)
        (moverz, rotate degree)
    '''
    if 'message' not in cache:
        return
    message = cache['message']
    cache.pop('message')
    
    message_execute_func_mapping = {
        'get_image': run_get_image,
    }
    message_execute_func_mapping[message[0]](cache, local_vars, *message[1:])
    # generate action
    generate_action(cache, local_vars)

def run_get_image(cache: dict, local_vars: dict):
    robot: Union[FetchAdvanced, TiagoAdvanced] = local_vars['robot']
    image_res = robot.get_image()
    rgb_image, depth_image = image_res['rgb'], image_res['depth']
    print(convert_ndarray_in_dict_to_shape(image_res))
    # write rgb_image & depth_image to shared memory
    # shared_memory = cache['shared_memory']
    # ...

def run_movex(cache: dict, local_vars: dict, distance: float):
    return

def generate_action(cache: dict, local_vars: dict):
    return

def keyboardInput_to_delta(inp, delta=0.01):
    marker_mapping = {
        carb.input.KeyboardInput.W: np.array([delta, 0, 0]),
        carb.input.KeyboardInput.S: np.array([-delta, 0, 0]),
        carb.input.KeyboardInput.DOWN: np.array([0, 0, -delta]),
        carb.input.KeyboardInput.UP: np.array([0, 0, delta]),
        carb.input.KeyboardInput.A: np.array([0, delta, 0]),
        carb.input.KeyboardInput.D: np.array([0, -delta, 0]),
    }
    move_mapping = {
        carb.input.KeyboardInput.I: np.array([0, 1]), # forward
        carb.input.KeyboardInput.K: np.array([0, -1]), # backward
        carb.input.KeyboardInput.J: np.array([1, 0]), # left
        carb.input.KeyboardInput.L: np.array([-1, 0]), # right
    }
    
    marker_delta = marker_mapping.get(inp, None)
    move_delta = move_mapping.get(inp, None)
    return marker_delta, move_delta

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
