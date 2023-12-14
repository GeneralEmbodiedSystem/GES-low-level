"""
Example script demo'ing robot manipulation control with grasping.
"""
from math import ceil
from typing import Union, Dict, List

import numpy as np

import omnigibson as og
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from omnigibson.utils.ui_utils import choose_from_options, KeyboardRobotController
from omnigibson.utils.asset_utils import get_available_g_scenes, get_available_og_scenes
from omnigibson.robots import FetchAdvanced

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
    
    log.warning(f"vision_res: {omit_convert_dict(robot.get_image())}")
    
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
    max_steps = -1 if not short_exec else 100
    step = 0
    action_random_selection = False
    
    def turn(deg, cur_step):
        """degrees to rotate base

        Args:
            deg (int): -180 ~ 180. [-180, -1] means turn right. [1, 180] means turn left.
            cur_step (int): current step (start time)
        """
        left_or_right = 1 if deg > 0 else -1  # 1 means turn left, -1 means turn right
        act_2 = left_or_right * 0.341  # 1 degrees per step
        continue_steps = abs(deg)
        action = np.zeros(robot.action_dim)
        for _ in range(continue_steps):
            action[2] = act_2
            env.step(action)
        print("turn", deg, "degree, from step", cur_step, "to", cur_step + continue_steps)
        return cur_step + continue_steps
    
    def movex(s, cur_step):
        """distance to move along x axis

        Args:
            s (int): distance to move along x axis. [-5, -0.1] means move back. [0.1, 5] means move forward.
            cur_step (int): _description_
        """
        forward_or_back = 1 if s > 0 else -1
        act_0 = forward_or_back * 0.1
        continue_steps = ceil(abs(s) / 0.0024)  # a little far from the real distance
        action = np.zeros(robot.action_dim)
        for _ in range(continue_steps):
            action[0] = act_0
            env.step(action)
        print("move", s, "m, from step", cur_step, "to", cur_step + continue_steps)
        return cur_step + continue_steps
    
    def movey(s, cur_step):
        """distance to move along y axis

        Args:
            s (int): distance to move along y axis. [-5, -0.1] means move right. [0.1, 5] means move left.
            cur_step (int): _description_
        """
        init_y = robot.get_position()[1]
        left_or_right = 1 if s > 0 else -1
        act_1 = left_or_right * 0.2
        # 这个y轴算不准速度...。 而且有bug，最多走80/130/?步，就走不了了。以下记录了每种速度至多走?步，走多远
        # 80 * 0.1act -> ~0.172m
        # 130 * 0.2act -> ~0.618m
        # 28 * 0.3act -> ~0.172m
        continue_steps = 0
        action = np.zeros(robot.action_dim)
        while True:
            action[1] = act_1
            env.step(action)
            continue_steps += 1
            print("continue_steps robot.get_position()[1], init_y", continue_steps, robot.get_position()[1], init_y)
            if left_or_right * (robot.get_position()[1] - init_y) >= abs(s):
                break
        print("move", s, "m, from step", cur_step, "to", cur_step + continue_steps)
        return cur_step + continue_steps

    while step != max_steps:
        # action = action_generator.get_random_action() if action_random_selection else action_generator.get_teleop_action()
        # for _ in range(10):
        #     env.step(action)
        #     step += 1
        # if step == 50:  # test turn()
        #     step = turn(-90, step)
        # if step == 50: # test movex()
        #     step = movex(0.4, step)
        if step == 50: # test movey()
            step = movey(1.5, step)
        action = np.zeros(robot.action_dim)
        # if step < 50:  # measure the distance of 1 step in y axis
        #     action[1] = 0.1
        # elif step < 100:
        #     action[1] = -0.1
        env.step(action)
        step += 1
        print("step", step)
        print("robot", robot.get_position())
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


if __name__ == "__main__":
    main()
