#!/usr/bin/env python

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

# Params to be set as needed.
SCENES = ["Beechwood_0_int"]
OUTPUT_DIR = os.path.join(os.path.expanduser("~"), "Desktop")
NUM_STEPS = 2000
gm.DEFAULT_VIEWER_WIDTH = 720
gm.DEFAULT_VIEWER_HEIGHT = 720  # high-res

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


def benchmark_scene(scene_name, optimized=False, import_robot=True, random_selection=False):
    assets_version = get_og_assets_version()
    print("assets_version", assets_version)
    scene = InteractiveTraversableScene(
        scene_model=scene_name,
        load_object_categories=["floors", "walls", "ceilings"],
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
    og.sim.viewer_camera.set_position_orientation(
        position=np.array([1.14539, 1.00671, 0.68562]),
        orientation=np.array([0.21118, 0.65232, 0.69255, 0.22419]), # here is xyzw, in isaac sim wxyz
    )

    # # Hide main viewport
    # og.sim.viewer_visibility = False

    if import_robot:
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
                }
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
        if scene_name == "restaurant_hotel":
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
            og.log.info("Querying joint configuration to current marker position")
            # Grab the joint positions in order to reach the desired pose target
            joint_pos = ik_solver.solve(
                target_pos=pos,
                target_quat=quat,
                max_iterations=max_iter,
            )
            if joint_pos is not None:
                og.log.info("Solution found. Setting new arm configuration.")
            else:
                og.log.info("EE position not reachable.")
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
        # command = robot.get_eef_position()
        # command = np.array([0.08, 0.05, 0.45])
        # command = np.array([0.34555, 0.05, 0.56099])
        command = np.array([0.34555, 0.25, 0.80099])
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
                    execute_ik(pos=command)
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
                        print(f"marker pos: {command}")
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
    
    while not exit_now:
        execute_ik(pos=command)
        # og.sim.step()
    
    og.app.close()
    return
    
    fps = []
    physics_fps = []
    render_fps = []
    obj_awake = []
    for i in range(NUM_STEPS):
        start = time.time()
        if import_robot:
            # Apply random actions.
            robot.apply_action(robot.action_space.sample())
        og.sim.step(render=False)
        
        # TODO: still buggy
        # action = action_generator.get_random_action() if random_selection else action_generator.get_teleop_action()
        # for _ in range(10):
        #     robot.apply_action(action)
        #     og.sim.step(render=False)
            
        physics_end = time.time()

        og.sim.render()
        end = time.time()

        if i % 100 == 0:
            print("Elapsed time: ", end - start)
            print("Render Frequency: ", 1 / (end - physics_end))
            print("Physics Frequency: ", 1 / (physics_end - start))
            print("Step Frequency: ", 1 / (end - start))
        fps.append(1 / (end - start))
        physics_fps.append(1 / (physics_end - start))
        render_fps.append(1 / (end - physics_end))
        # obj_awake.append(sim.body_links_awake)

    plt.figure(figsize=(7, 25))
    ax = plt.subplot(6, 1, 1)
    plt.hist(render_fps)
    ax.set_xlabel("Render fps")
    ax.set_title(
        "Scene {} version {}\noptimized {} num_obj {}\n import_robot {}".format(
            scene_name, assets_version, optimized, scene.n_objects, import_robot
        )
    )
    ax = plt.subplot(6, 1, 2)
    plt.hist(physics_fps)
    ax.set_xlabel("Physics fps")
    ax = plt.subplot(6, 1, 3)
    plt.hist(fps)
    ax.set_xlabel("Step fps")
    ax = plt.subplot(6, 1, 4)
    plt.plot(render_fps)
    ax.set_xlabel("Render fps with time, converge to {}".format(np.mean(render_fps[-100:])))
    ax.set_ylabel("fps")
    ax = plt.subplot(6, 1, 5)
    plt.plot(physics_fps)
    ax.set_xlabel("Physics fps with time, converge to {}".format(np.mean(physics_fps[-100:])))
    ax.set_ylabel("fps")
    ax = plt.subplot(6, 1, 6)
    plt.plot(fps)
    ax.set_xlabel("Overall fps with time, converge to {}".format(np.mean(fps[-100:])))
    ax.set_ylabel("fps")
    # TODO! Reading objects' wake state not available yet from omniverse
    # ax = plt.subplot(6, 1, 6)
    # plt.plot(obj_awake)
    # ax.set_xlabel("Num object links awake, converge to {}".format(np.mean(obj_awake[-100:])))
    plt.tight_layout()
    plt.savefig(os.path.join(
        OUTPUT_DIR,
        "scene_benchmark_{}_o_{}_r_{}.pdf".format(scene_name, optimized, import_robot)))

    from IPython import embed; embed()

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
