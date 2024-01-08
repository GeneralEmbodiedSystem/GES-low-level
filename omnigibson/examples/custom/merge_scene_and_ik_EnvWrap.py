"""
Example script demo'ing robot manipulation control with grasping.
"""
from typing import Union, Dict, List

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
from omnigibson.objects import PrimitiveObject

import carb
import omni

from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.usd

from pxr import Gf


from omnigibson.utils.ui_utils import create_module_logger
log = create_module_logger(module_name=__name__)

from utils.common import omit_convert_dict

GRASPING_MODES = dict(
    sticky="Sticky Mitten - Objects are magnetized when they touch the fingers and a CLOSE command is given",
    physical="Physical Grasping - No additional grasping assistance applied",
)

# Don't use GPU dynamics and Use flatcache for performance boost
gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True


def get_pos_by_prim_path(prim_path: str):
    '''
    @param prim_path: e.g. /world/robot/eyes
    @return position/translate: np.ndarray([x, y, z])
    '''
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    translate: Gf.Vec3d = matrix.ExtractTranslation()
    return np.array(translate)


def main(random_selection=False, headless=False, short_exec=False):
    """
    Robot grasping mode demo with selection
    Queries the user to select a type of grasping mode
    """
    og.log.info(f"Demo {__file__}\n    " + "*" * 80 + "\n    Description:\n" + main.__doc__ + "*" * 80)

    # Choose type of grasping
    grasping_mode = choose_from_options(options=GRASPING_MODES, name="grasping mode", random_selection=random_selection)
    print(f"grasping_mode: {grasping_mode}")

    # Create environment configuration to use
    scene_type = "InteractiveTraversableScene"  # fix
    scenes = get_available_og_scenes() if scene_type == "InteractiveTraversableScene" else get_available_g_scenes()
    scene_model = choose_from_options(options=scenes, name="scene model", random_selection=random_selection)
    
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
        prim_path="/World/robot",
        name="robot",
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

    # Define objects to load
    # 整体上x 和 y小一些，可以让robot和物体放到客厅里去。现在的房间太小了
    apple_init_pos = [0.5, -0.1, 0.97]
    objects_cfg = [ 
        # {
        #     "type": "USDObject", 
        #     "name": "ghost_stain", 
        #     "usd_path": f"{gm.ASSET_PATH}/models/stain/stain.usd",
        #     "category": "stain", 
        #     "visual_only": True, 
        #     "scale": [2.0, 1.0, 2.0], 
        #     "position": [3.0, 0, 2.0], 
        #     "orientation": [0, 0, 0, 1.0], 
        # },
        # {
        #     "type": "DatasetObject", 
        #     "name": "delicious_apple",
        #     "category": "apple",
        #     "model": "agveuv", 
        #     "position": [1.23, -0.1, 0.97],  # 两个物体的坐标还要大一点，放到场景自带的桌子上
        # },
        dict(
            type="DatasetObject",
            name="delicious_apple",
            category="apple",
            model="agveuv",
            size=0.04,
            position=apple_init_pos,
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
        # dict(
        #     type="DatasetObject",
        #     name="table",
        #     category="breakfast_table",
        #     model="lcsizg",
        #     bounding_box=[0.5, 0.5, 0.8],
        #     fit_avg_dim_volume=False,
        #     fixed_base=True,
        #     position=[0.7, -0.1, 0.6],
        #     orientation=[0, 0, 0.707, 0.707],
        # ),  # table
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
            position=[0.53, -0.2, 0.97],
        ),  # box
    ]

    # Compile config
    cfg = dict(scene=scene_cfg, robots=[robot0_cfg], objects=objects_cfg)

    # Create the environment
    env = og.Environment(configs=cfg, action_timestep=1/60., physics_timestep=1/60.)

    # Update the simulator's viewer camera's pose so it points towards the robot
    og.sim.viewer_camera.set_position_orientation(
        position=np.array([1.20465,  1.51194,  2.14789]),
        orientation=np.array([0.144,  0.424,  0.846, 0.28757]),
    )
    og.sim.step()

    # Reset the robot
    robot:FetchAdvanced = env.robots[0]

    # --------------------ik controller to reach the target object--------------------
    control_idx = np.concatenate([robot.trunk_control_idx, robot.arm_control_idx[robot.default_arm]])
    ik_solver = IKSolver(
        robot_description_path=robot.robot_arm_descriptor_yamls[robot.default_arm],
        robot_urdf_path=robot.urdf_path,
        default_joint_pos=robot.get_joint_positions()[control_idx],
        eef_name=robot.eef_link_names[robot.default_arm],
    )
    # Define a helper function for executing specific end-effector commands using the ik solver
    def execute_ik(pos, quat=None, max_iter=100):
        # note: pos is world-frame pos
        # og.log.info("Querying joint configuration to current marker position")
        # Grab the joint positions in order to reach the desired pose target
        joint_pos = ik_solver.solve(
            target_pos=pos - robot.get_position(),  # notice the pos!
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

    robot.set_position([-0.08, 0, 0])
    robot.reset()
    robot.keep_still()

    

    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = 720
            sensor.image_width = 720
    
    log.warning(f"vision_res: {omit_convert_dict(robot.get_cam_obs())}")
    
    # return



    # Create teleop controller
    action_generator = KeyboardRobotController(robot=robot)

    # Print out relevant keyboard info if using keyboard teleop
    action_generator.print_keyboard_teleop_info()

    # Other helpful user info
    print("Running demo with grasping mode {}.".format(grasping_mode))
    print("Press ESC to quit")

    # Loop control until user quits
    max_steps = -1 if not short_exec else 100
    step = 0
    while step != max_steps:
        # print("env.objects_config", env.objects_config)  # no position here
        apple_pos = get_pos_by_prim_path("/World/delicious_apple/base_link") - apple_init_pos  # notice the pos!
        print("apple pos", apple_pos)  # apple
        # action = action_generator.get_random_action() if random_selection else action_generator.get_teleop_action()
        # for _ in range(10):
        #     env.step(action)
        #     step += 1
        execute_ik(apple_pos)

    # Always shut down the environment cleanly at the end
    env.close()


if __name__ == "__main__":
    main()
