import argparse
import yaml
import numpy as np
import pkgutil


import omnigibson as og
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from omnigibson.tasks.behavior_task import BehaviorTask
from omnigibson.robots import TiagoAdvanced
from omnigibson.custom_env.configs import ENV_CONFIG_PATH
import bddl.activity_definitions
from omnigibson.utils.python_utils import list_directories

# activity_name = "boil_water"     # the activity you want to try, full list in activity_definitions
# activity_definition_id = 0                         # the specific definition you want to use. As of BEHAVIOR100 2021, this should always be 0.
# activity_instance_id = 0


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_file", "-c", type=str, default="behavior_cfg.yaml")
    parser.add_argument("--record", "-r", type=int, default=None)
    args = parser.parse_args()
    
    return args

def main(random_selection=False, headless=False, short_exec=False, args=get_args()):
    # behavior_activity_instance = BehaviorTask(activity_name=activity_name, activity_definition_id=activity_definition_id, activity_instance_id=activity_instance_id)

    # environment config, including scene, robot, and objects
    sim_config_path = ENV_CONFIG_PATH + "/" + args.config_file
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.load(f, Loader=yaml.FullLoader)
    
    # Create environment configuration to use
    scene_cfg = sim_config["scene_cfg"]

    robot0_cfg = sim_config["robot0_cfg"]

    # type of grasping
    grasping_mode = sim_config["robot0_cfg"]['grasping_mode']

    # empty object config
    objects_cfg = sim_config["objects_cfg"]

    task_cfg = sim_config["task_cfg"]
    """
    you can replace task_cfg["activity_name"] (in omnigibson/custom_env/configs/behavior_cfg.yaml) to any activity \in activities_list
    activity_list is from https://github.com/StanfordVL/bddl/tree/master/bddl/activity_definitions
    """
    activities_list = list_directories(list(bddl.activity_definitions.__path__)[0])
    print(activities_list)

    # Compile config
    cfg = dict(
        scene=scene_cfg, 
        robots=[robot0_cfg], 
        objects=objects_cfg, 
        task=task_cfg
    )
    print("cfg setup")
    
    # Make sure sim is stopped
    og.sim.stop()

    # Make sure GPU dynamics are enabled (GPU dynamics needed for cloth) and no flatcache
    gm.ENABLE_OBJECT_STATES = True  # Make sure object states are enabled
    gm.USE_GPU_DYNAMICS = True
    gm.ENABLE_FLATCACHE = False
    
    # Create the environment
    env = og.Environment(configs=cfg, action_timestep=1/60., physics_timestep=1/60.)
    print("env setup")
    
    robot: TiagoAdvanced = env.robots[0]
    # robot.set_position(sim_config["other_init_cfg"]["robot_position"])  # robot0_cfg["position"] will be modified! DON'T USE IT!
    # robot.set_orientation(sim_config["other_init_cfg"]["robot_orientation"]) 
    # robot.reset()
    # robot.lift_trunk_body()
    # og.sim.step()
    
    # limit the max joint velocities of the robot
    robot.max_joint_velocities[robot.ik_control_idx] *= 1
    robot.max_joint_efforts[robot.ik_control_idx] *= 1
    
    # Make the robot's camera(s) high-res
    for sensor in robot.sensors.values():
        if isinstance(sensor, VisionSensor):
            sensor.image_height = sim_config["other_init_cfg"]["camera_resolution"]["image_height"]
            sensor.image_width = sim_config["other_init_cfg"]["camera_resolution"]["image_width"]
    og.sim.step()
    
    # robot = behavior_activity_instance.get_agent()
    # print("show_instruction", behavior_activity_instance.show_instruction())
    
    
    task: BehaviorTask = env.task
    print("task's instruction", task.show_instruction())
    entity0 = task.show_instruction()[2][0]  # it is BDDLEntity
    print("entity0.name", entity0.name) 
    print("entity0.wrapped_obj", entity0.wrapped_obj) 
    # env.step(np.zeros(robot.action_dim))
    # print(task.done, '\n', task.info, '\n', task.name, '\n', task.reward)
    # print(task.default_reward_config, '\n', task.default_termination_config)
    """summary of task :    
    task.name: BehaviorTask_assembling_gift_baskets_0_0
    task.info: {'reward': {'potential': {}, 'reward_breakdown': {'potential': 0.0}}, 'done': {'success': False, 'goal_status': {'satisfied': [], 'unsatisfied': [0, 1, 2, 3]}}}
    task.done is True/False, task.reward is a float number.
    task.info, task.done, task.reward are given after each env.step.
    
    task-related objects are stored in task.show_instruction()[2], which is a list of BDDLEntity (like entity0).
    entity0.name is the name of object(like "swiss_cheese_32"), each similar object has a different number.; 
    entity0.wrapped_obj is a DatasetObject. You can use it to get the object's position, orientation, etc.
    """
    
    # Other helpful user info
    # print("activity_name", task_cfg["activity_name"])
    print(f"Running activity {task_cfg['activity_name']} and scene model {scene_cfg.get('scene_model', None)}.")
    print("Use CTRL+C to quit")
    step = 0
    while True:
        action = np.zeros(robot.action_dim)
        for _ in range(10):
            obs, reward, done, info = env.step(action)
            step += 1

    # Clear the sim
    og.sim.clear()


if __name__ == "__main__":
    main()