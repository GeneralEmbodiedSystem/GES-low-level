import numpy as np

import os
from os.path import join, exists

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.objects import VisualCuboid, FixedCuboid

from omni.isaac.motion_generation import RmpFlow, ArticulationMotionPolicy
from omni.isaac.motion_generation.interface_config_loader import (
    get_supported_robot_policy_pairs,
    load_supported_motion_policy_config,
)


class FrankaRmpFlowExample():
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None

        self._dbg_mode = True
        
        self._my_world = World(stage_units_in_meters=1.0)
        self._my_world.scene.add_default_ground_plane()
        self._my_world.reset()

    def load_example_assets(self):
        # Add the Franka and target to the stage
        # The position in which things are loaded is also the position in which they

        robot_prim_path = "/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = self._my_world.scene.add(Articulation(robot_prim_path))

        # add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = self._my_world.scene.add(
            # XFormPrim(
            #     "/World/target",
            #     scale=[0.04, 0.04, 0.04],
            #     position=np.array([0.5, 0.0, 0.7]),
            #     orientation=euler_angles_to_quats([0, np.pi, 0]),
            # )
            FixedCuboid(
                "/World/target",
                name='target',
                size=0.02,
                # scale=[0.04, 0.04, 0.04],
                position=np.array([0.5, 0.0, 0.7]),
                orientation=euler_angles_to_quats([0, np.pi, 0]),
                color=np.array([1.0, 0.0, 0.0])
            )
        )

        self._obstacle = self._my_world.scene.add(
            FixedCuboid(
                "/World/obstacle",
                name='obstacle',
                size=0.05,
                # position=np.array([0.4, 0.0, 0.65]),
                position=np.array([0.4, 0.0, 4.65]),
                color=np.array([0.0, 0.0, 1.0])
            )
        )
        
        self._my_world.reset()

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._articulation, self._target, self._obstacle

    def setup(self):
        # Loading RMPflow can be done quickly for supported robots
        # print("Supported Robots with a Provided RMPflow Config:", list(get_supported_robot_policy_pairs().keys()))
        # rmp_config = load_supported_motion_policy_config("Franka", "RMPflow")
        # import pprint
        # pp = pprint.PrettyPrinter()
        # print(f'rmp_config:')
        # pp.pprint(rmp_config)

        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs/franka/rmpflow/franka_rmpflow_common.yaml
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf
        rmp_config_dir = join(mg_extension_path, "motion_policy_configs")
        rmp_config = {
            'end_effector_frame_name': 'right_gripper',
            'ignore_robot_state_updates': False,
            'maximum_substep_size': 0.00334,
            'rmpflow_config_path': rmp_config_dir + '/franka/rmpflow/franka_rmpflow_common.yaml',
            'robot_description_path': rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            'urdf_path': rmp_config_dir + '/franka/lula_franka_gen.urdf'
        }
        
        import pprint
        pp = pprint.PrettyPrinter()
        print(f'rmp_config:')
        pp.pprint(rmp_config)

        #Initialize an RmpFlow object
        self._rmpflow = RmpFlow(**rmp_config)
        self._rmpflow.add_obstacle(self._obstacle)

        if self._dbg_mode:
            self._rmpflow.set_ignore_state_updates(True)
            # self._rmpflow.visualize_collision_spheres()

            # Set the robot gains to be deliberately poor
            # bad_proportional_gains = self._articulation.get_articulation_controller().get_gains()[0]/50
            gains = self._articulation.get_articulation_controller().get_gains() # kps & kds
            print(f'gains: {gains}')
            bad_proportional_gains = gains[0]
            print(f'bad_proportional_gains: {bad_proportional_gains}')
            self._articulation.get_articulation_controller().set_gains(kps = bad_proportional_gains)

        #Use the ArticulationMotionPolicy wrapper object to connect rmpflow to the Franka robot articulation.
        self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation,self._rmpflow)

        self._my_world.reset()

    def update(self, step: float = 1):
        # Step is the time elapsed on this frame
        target_position, target_orientation = self._target.get_world_pose()

        self._rmpflow.set_end_effector_target(
            target_position, target_orientation
        )

        # Track any movements of the cube obstacle
        self._rmpflow.update_world()

        #Track any movements of the robot base
        robot_base_translation,robot_base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(robot_base_translation,robot_base_orientation)

        action = self._articulation_rmpflow.get_next_articulation_action(step)
        self._articulation.apply_action(action)
        
        self._my_world.step()

    def reset(self):
        # Rmpflow is stateless unless it is explicitly told not to be
        if self._dbg_mode:
            # RMPflow was set to roll out robot state internally, assuming that all returned joint targets were hit exactly.
            self._rmpflow.reset()
            # self._rmpflow.visualize_collision_spheres()

        self._my_world.reset()




# add noise to the camera
# /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.ros_bridge/camera_noise.py
# learn to get perspective camera prim