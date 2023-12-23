"""
Example script demo'ing robot manipulation control with grasping.
"""
# TODO: adjust arm default joint position
# TODO: make sticky mode valid
# TODO: high position first, then fix hand not rotate, then reach the goal
from typing import Union, Dict, List, Any

import pprint
import cv2

import numpy as np

import omnigibson as og
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from omnigibson.utils.ui_utils import choose_from_options, KeyboardRobotController
from omnigibson.utils.asset_utils import get_available_g_scenes, get_available_og_scenes
from omnigibson.robots import FetchAdvanced

from omnigibson.robots.turtlebot import Turtlebot
from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.scenes.interactive_traversable_scene import InteractiveTraversableScene
from omnigibson.utils.asset_utils import get_og_assets_version
from omnigibson.utils.control_utils import IKSolver
from omnigibson.objects import PrimitiveObject, DatasetObject, LightObject

import carb
import omni

from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.physx import get_physx_interface
import omni.usd

from pxr import Gf

from utils.prim_utils import get_pos_by_prim_path, get_pos_by_prim_path_physxApi
from utils.simulator_utils import custom_step
from utils.skill_utils import lift_trunk_body, execute_grasp, execute_ik, execute_look_at, \
    execute_record, execute_release

from omnigibson.utils.ui_utils import create_module_logger
log = create_module_logger(module_name=__name__)

pp = pprint.PrettyPrinter()

# from utils.common import omit_convert_dict

def omit_convert_dict(x: Union[Any, Dict]):
    if not isinstance(x, dict):
        return type(x)
    omit_print_res = dict()
    for k, v in x.items():
        omit_print_res[k] = omit_convert_dict(v)
    return omit_print_res

GRASPING_MODES = dict(
    sticky="Sticky Mitten - Objects are magnetized when they touch the fingers and a CLOSE command is given",
    physical="Physical Grasping - No additional grasping assistance applied",
)

# Don't use GPU dynamics and Use flatcache for performance boost
gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True


def main(random_selection=False, headless=False, short_exec=False):
    """
    Robot grasping mode demo with selection
    Queries the user to select a type of grasping mode
    """
    og.log.info(f"Demo {__file__}\n    " + "*" * 80 + "\n    Description:\n" + main.__doc__ + "*" * 80)

    # Choose type of grasping
    # grasping_mode = choose_from_options(options=GRASPING_MODES, name="grasping mode", random_selection=random_selection)
    # grasping_mode = 'sticky'

    # Create environment configuration to use
    scene_type = "InteractiveTraversableScene"  # fix
    # scenes = get_available_og_scenes() if scene_type == "InteractiveTraversableScene" else get_available_g_scenes()
    # scene_model = choose_from_options(options=scenes, name="scene model", random_selection=random_selection)
    scene_model = 'Beechwood_0_int'
    
    # create scene
    scene_cfg = dict(
        type=scene_type,
        scene_model=scene_model,
        load_object_categories=["floors", "walls", "ceilings"],
    )



    # Define objects to load
    # 整体上x 和 y小一些，可以让robot和物体放到客厅里去。现在的房间太小了
    position_mapping = {
        # 'delicious_apple': np.array([0.65, -0.1, 0.310]),
        'delicious_apple': np.array([1.5, -0.1, 0.310]),
        'alarm_clock': np.array([1.53, 0.1, 0.97]),
        'water_bottle': np.array([1.33, -0.5, 0.97]),
        'brilliant_light': np.array([3.0, 3.0, 4.0]),
        'box': np.array([0.61, -0.1, 0.2995]),
        # 'viewer_camera': np.array([1.20465, 1.51194, 2.14789]),
        'viewer_camera': np.array([1.71316, -0.39746, 0.78216]),
        'robot': np.array([0, 0, 0.0]),
        'breakfast_table': np.array([0.7, -0.1, 0.22]),
        'trash_can': np.array([0.7, 0.4, 0.22]),
    }

    orientation_mapping = {
        # 'viewer_camera': np.array([0.144, 0.424, 0.846, 0.28757]),
        'viewer_camera': np.array([0.43339, 0.31573, 0.49696, 0.68229]),
        'breakfast_table': np.array([0, 0, 0.707, 0.707]),
    } # object_name to position
    
    # create robot
    robot0_cfg = {
        # type="Fetch",
        'name': "robot",
        'type': "TiagoAdvanced",
        'obs_modalities': ["scan", "rgb", "depth"],
        'action_type': "continuous",
        'action_normalize': True,
        'grasping_mode': 'sticky',
        'self_collisions': False,
        'position': position_mapping['robot'],
        'prim_path': "/World/robot",
        'fixed_base': False,
        'controller_config': {
            "camera": {
                "name": "JointController",
                "motor_type": "position",
            },
            "base": {
                "name": "JointController",
                "motor_type": "velocity",
            },
            "arm_left": {
                "name": "JointController",
                "motor_type": "position",
            },
            # "arm_right": {
            #     "name": "JointController",
            #     "motor_type": "position",
            # }
        }
    }
    
    # create objects
    objects_cfg = [
        {
            'type': "DatasetObject",
            'name': "delicious_apple",
            'category': "apple",
            'model': "agveuv",
            'size': 0.02,
            'fixed_base': False,
            'position': position_mapping['delicious_apple'],
        },
        {
            'type': "DatasetObject",
            'name': "alarm_clock",
            'category': "alarm_clock",
            'model': "cvknrh",
            'size': 0.06,
            'fixed_base': False,
            'position': position_mapping['alarm_clock'],
        },
        {
            'type': "DatasetObject",
            'name': "water_bottle",
            'category': "water_bottle",
            'model': "ackxiy",
            'size': 0.06,
            'fixed_base': False,
            'position': position_mapping['water_bottle'],
        },
        {
            'type': "LightObject", 
            'name': "brilliant_light",
            'light_type': "Sphere", 
            'intensity': 50000, 
            'radius': 0.1, 
            'fixed_base': False,
            'position': position_mapping['brilliant_light'],
        },
        {
            'type': "PrimitiveObject",
            'name': "box",
            'primitive_type': "Cube",
            # 'visual_only': True,
            'rgba': [1.0, 0, 0, 1.0],
            'size': 0.05,
            'fixed_base': False,
            'position': position_mapping['box'],
        },
        {
            'type': "DatasetObject",
            'name': "breakfast_table",
            'category': "breakfast_table",
            'model': "lcsizg",
            # 'visual_only': True,
            'bounding_box': [0.5, 0.5, 0.2745],
            'fit_avg_dim_volume': False,
            'fixed_base': True,
            'position': position_mapping['breakfast_table'],
            'orientation': orientation_mapping['breakfast_table'],
        },
        {
            'type': "DatasetObject",
            'name': "trash_can",
            'category': "trash_can",
            'model': "aefcem",
            'bounding_box': [0.25, 0.25, 0.2],
            'size': 0.06,
            # 'visual_only': True,
            'position': position_mapping['trash_can'],
        },
    ]
    def object_index(object_name: str):
        for i, foo in enumerate(objects_cfg):
            if foo['name'] == object_name:
                return i
        return None

    # Create the environment
    env = og.Environment(
        configs={
            'scene': scene_cfg, 
            'robots': [robot0_cfg], 
            'objects': objects_cfg
        },
        action_timestep=1/60.,
        physics_timestep=1/60.,
    )

    # Update the simulator's viewer camera's pose so it points towards the robot
    og.sim.viewer_camera.set_position_orientation(
        position=np.array(position_mapping['viewer_camera']),
        orientation=np.array(orientation_mapping['viewer_camera']),
    )
    og.sim.step()

    # Retrive the robot and objects
    robot: TiagoAdvanced = env.robots[0]
    # delicious_apple: DatasetObject = env.objects[object_index('delicious_apple')]
    # alarm_clock: DatasetObject = env.objects[object_index('alarm_clock')]
    # water_bottle: DatasetObject = env.objects[object_index('water_bottle')]
    # brilliant_light: LightObject = env.objects[object_index('brilliant_light')]
    # box: PrimitiveObject = env.objects[object_index('box')]
    
    # set robot and objects' init position
    # robot.set_position(position=position_mapping['robot'])
    robot.reset()
    robot.keep_still()
    og.sim.step()
    # for object in ['delicious_apple', 'alarm_clock', 'water_bottle', 'brilliant_light', 'box']:
    #     eval(object).set_position(position=position_mapping[object])
    # og.sim.step()

    # Create teleop controller
    # action_generator = KeyboardRobotController(robot=robot)

    # grasp(robot, 'left')
    # og.sim.step()
    
    # lift the robot body
    lift_trunk_body(robot)
    og.sim.step()

    # --------------------ik controller to reach the target object--------------------
    control_idx = np.concatenate([robot.trunk_control_idx, robot.arm_control_idx[robot.default_arm]])
    # print(f'default joint pos: {robot.get_joint_positions()[control_idx]}')
    ik_solver = IKSolver(
        robot_description_path=robot.robot_arm_descriptor_yamls[robot.default_arm],
        robot_urdf_path=robot.urdf_path,
        default_joint_pos=robot.get_joint_positions()[control_idx],
        eef_name=robot.eef_link_names[robot.default_arm],
    )

    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = 720
            sensor.image_width = 720
    
    # log.warning(f"vision_res: {omit_convert_dict(robot.get_image())}")


    # # Print out relevant keyboard info if using keyboard teleop
    # action_generator.print_keyboard_teleop_info()

    # Other helpful user info
    # print("Running demo with grasping mode {}.".format(grasping_mode))
    # print("Press ESC to quit")

    # Loop control until user quits
    # max_steps = -1 if not short_exec else 100
    max_steps = 1600
    record_flag = True
    step = 0
    buffer = dict(); buffer['tgt_joint_pos'] = dict()
    while step != max_steps:
        # print(f'now joint pos: {robot.get_joint_positions()[control_idx]}')
        # print("env.objects_config", env.objects_config)  # no position here
        # apple_pos = get_pos_by_prim_path("/World/delicious_apple/base_link") - position_mapping['delicious_apple']  # notice the pos!
        apple_pos = get_pos_by_prim_path_physxApi("/World/delicious_apple/base_link") # this is world position
        box_pos = get_pos_by_prim_path_physxApi("/World/box/base_link") # this is world position
        robot_pos = get_pos_by_prim_path_physxApi("/World/robot/base_link") # this is world position
        # print(f'robot_pos: {robot_pos}')
        tgt_pos = box_pos
        # print("tgt pos", tgt_pos)
        above_tgt_pos1 = tgt_pos.copy(); above_tgt_pos1[-1] += 0.1
        above_tgt_pos2 = tgt_pos.copy()
        
        # grasp(robot)
        
        action = np.zeros(robot.action_dim)
        if step == 100:
            execute_ik(ik_solver, robot, above_tgt_pos1, buffer)
        elif step == 600:
            execute_ik(ik_solver, robot, above_tgt_pos2, buffer)
        elif step == 750:
            execute_grasp(robot, buffer, 'left')
        elif step == 900:
            trash_can_pos = get_pos_by_prim_path_physxApi("/World/trash_can/base_link")
            trash_can_above_pos = trash_can_pos.copy()
            trash_can_above_pos[-1] = 0.5
            execute_ik(ik_solver, robot, trash_can_above_pos, buffer)
        elif step == 1400:
            execute_release(robot, buffer, 'left')
        
        # elif step == 1500:
        #     execute_ik(ik_solver, robot, tgt_pos, buffer)
        
        # execute_ik(ik_solver, robot, tgt_pos, buffer)
        # execute_grasp(robot, buffer, 'left')
        # og.log.info(f'mk action_idx: {action_idx}, action_val: {action_val}')
        # print(f'joint pos: {robot.get_joint_positions()}')
        
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
        print(f'step: {step}')
        
        if record_flag:
            if step == max_steps:
                execute_record(og.sim.viewer_camera, buffer, './viewer_camera.mp4', 60, True)
            else:
                execute_record(og.sim.viewer_camera, buffer)

    # Always shut down the environment cleanly at the end
    env.close()

def extract_valid_pos_and_val(action):
    idx_valid = action.nonzero()[0]
    return idx_valid, action[idx_valid]

if __name__ == "__main__":
    main()
