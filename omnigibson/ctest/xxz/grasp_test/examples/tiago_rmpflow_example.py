import numpy as np

import os
from os.path import join, exists

from copy import deepcopy

# import omnigibson as og
# from omnigibson.macros import gm
# from omnigibson.sensors import VisionSensor
# from omnigibson.robots.turtlebot import Turtlebot
# from omnigibson.robots.tiago import Tiago
# from omnigibson.robots.tiago_advanced import TiagoAdvanced
# from omnigibson.robots import FetchAdvanced
# from omnigibson.scenes.interactive_traversable_scene import InteractiveTraversableScene
# from omnigibson.scenes.scene_base import Scene
# from omnigibson.utils.asset_utils import get_og_assets_version
# from omnigibson.utils.ui_utils import choose_from_options, KeyboardRobotController
# from omnigibson.utils.control_utils import IKSolver
# from omnigibson.objects import PrimitiveObject

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


class TiagoRmpFlowExample():
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None

        self._dbg_mode = True
        
        self._visual_out = True
        
        self._tiago_assets_root_path = '/home/mad0g4/Desktop/code/GES-low-level/omnigibson/ctest/mk/grasp_test/configs/rmpflow/tiago'
        
        self._tiago_dof_names = [
            'torso_lift_joint', 
            'arm_left_1_joint', # 1
            'arm_right_1_joint', 
            'head_1_joint', 
            'arm_left_2_joint', # 4
            'arm_right_2_joint', 
            'head_2_joint', 
            'arm_left_3_joint', # 7
            'arm_right_3_joint', 
            'base_footprint_rz_joint', 
            'arm_left_4_joint', # 10
            'arm_right_4_joint', 
            'base_footprint_ry_joint', 
            'arm_left_5_joint', # 13
            'arm_right_5_joint', 
            'base_footprint_rx_joint', 
            'arm_left_6_joint', # 16
            'arm_right_6_joint', 
            'base_footprint_z_joint', 
            'arm_left_7_joint', # 19
            'arm_right_7_joint', 
            'base_footprint_y_joint', 
            'base_footprint_x_joint', 
            'gripper_left_left_finger_joint', 
            'gripper_left_right_finger_joint', 
            'gripper_right_left_finger_joint', 
            'gripper_right_right_finger_joint'
        ]
        
        self._left_arm_indices = np.array([1, 4, 7, 10, 13, 16, 19])
        # self._left_arm_indices = np.array([0, 1, 4, 7, 10, 13, 16, 19])
        # self._right_arm_indices = self._left_arm_indices + 1
        self._left_gripper_indices = np.array([23, 24])
        
        self._my_world = World(stage_units_in_meters=1.0)
        self._my_world.scene.add_default_ground_plane()
        self._my_world.reset()
        

    def load_example_assets(self):
        # Add the Franka and target to the stage
        # The position in which things are loaded is also the position in which they

        robot_prim_path = "/World/tiago"
        path_to_robot_usd = join(
            self._tiago_assets_root_path, 
            'tiago_dual_omnidirectional_stanford_33.usd'
        )

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = self._my_world.scene.add(
            Articulation(
                prim_path=robot_prim_path,
                name='tiago',
                position=[0.0, 0.0, 0.0],
            )
        )
        
        self._my_world.reset()
        
        robot_default_joint_state = self._articulation.get_joints_default_state()
        robot_default_joint_state.positions[self._left_gripper_indices] = np.array([0.045, 0.045])
        self._articulation.set_joints_default_state(
            robot_default_joint_state.positions, 
            robot_default_joint_state.velocities, 
            robot_default_joint_state.efforts, 
        )
        # self._articulation.set_joint_positions(, self._left_gripper_indices)
        

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
                # position=np.array([0.5, 0.0, 0.7]),
                # position=np.array([0.3657, 0.56207, 0.7]),
                # position=np.array([0.56539, -0.20931, 0.09604]), # RIGHT FRONT
                position=np.array([0.18518, 0.65411, 0.32218]), # RIGHT LEFT
                # position=np.array([-0.32251, 0.45554, 0.24932]),
                orientation=euler_angles_to_quats([0, np.pi, 0]),
                color=np.array([1.0, 0.0, 0.0])
            )
        )

        # self._obstacle = self._my_world.scene.add(
        #     FixedCuboid(
        #         "/World/obstacle",
        #         name='obstacle',
        #         size=0.05,
        #         position=np.array([0.4, 0.0, 0.65]),
        #         color=np.array([0.0, 0.0, 1.0])
        #     )
        # )
        
        
        self._my_world.reset()
        
        # print(f'dof_properties: {self._articulation.dof_properties}')

        # Return assets that were added to the stage so that they can be registered with the core.World
        # return self._articulation, self._target, self._obstacle

    def setup(self):
        # Loading RMPflow can be done quickly for supported robots
        # print("Supported Robots with a Provided RMPflow Config:", list(get_supported_robot_policy_pairs().keys()))
        # rmp_config = load_supported_motion_policy_config("Franka", "RMPflow")
        # import pprint
        # pp = pprint.PrettyPrinter()
        # print(f'rmp_config:')
        # pp.pprint(rmp_config)

        # mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs/franka/rmpflow/franka_rmpflow_common.yaml
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml
        # /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf
        # rmp_config_dir = join(mg_extension_path, "motion_policy_configs")
        rmp_config = {
            'end_effector_frame_name': 'gripper_left_grasping_frame',
            'ignore_robot_state_updates': False,
            # 'maximum_substep_size': 0.00334,
            'maximum_substep_size': 0.05,
            'rmpflow_config_path': join(self._tiago_assets_root_path, 'tiago_rmpflow_common.yaml'),
            'robot_description_path': join(self._tiago_assets_root_path, 'tiago_dual_omnidirectional_stanford_left_arm_descriptor.yaml'),
            'urdf_path': join(self._tiago_assets_root_path, 'tiago_dual_omnidirectional_stanford.urdf'),
        }
        
        import pprint
        pp = pprint.PrettyPrinter()
        print(f'rmp_config:')
        pp.pprint(rmp_config)

        #Initialize an RmpFlow object
        self._rmpflow = RmpFlow(**rmp_config)
        # self._rmpflow.add_obstacle(self._obstacle)

        if self._dbg_mode:
            self._rmpflow.set_ignore_state_updates(True)
            if self._visual_out:
                self._rmpflow.visualize_collision_spheres()

            # Set the robot gains to be deliberately poor
            # bad_proportional_gains = self._articulation.get_articulation_controller().get_gains()[0] / 50
            # print(f'gains: {self._articulation.get_articulation_controller().get_gains()}')
            kps = np.array(
                # [1200000., 120000., 1200000., 120000., 5000., 3000., 10000., 1200., 1200.]
                [1200000., 120000., 5000., 3000., 10000., 1200., 1200.]
            )
            kds = np.array(
                # [600000., 60000., 300000., 30000., 3000., 3000., 3000., 1000., 1000.]
                [300000., 30000., 3000., 3000., 3000., 1000., 1000.]
            )
            now_gains = deepcopy(self._articulation.get_articulation_controller().get_gains())
            now_gains[0][self._left_arm_indices] = kps
            now_gains[1][self._left_arm_indices] = kds
            self._articulation.get_articulation_controller().set_gains(
                kps=now_gains[0], kds=now_gains[1]
            )

        #Use the ArticulationMotionPolicy wrapper object to connect rmpflow to the Franka robot articulation.
        self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self._rmpflow)

        self._my_world.reset()

    def update(self, step: float = 1):
        # Step is the time elapsed on this frame
        target_position, target_orientation = self._target.get_world_pose()
        
        # print(f'target_position: {target_position}')

        self._rmpflow.set_end_effector_target(
            target_position, target_orientation
        )

        # Track any movements of the cube obstacle
        self._rmpflow.update_world()

        #Track any movements of the robot base
        robot_base_translation,robot_base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        action = self._articulation_rmpflow.get_next_articulation_action(step)
        action.joint_indices = self._left_arm_indices
        # action.joint_velocities = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        # action.joint_indices = np.array([1, 4, 7, 10, 13, 16, 32])
        # print(f'tgt joint pos: {action.joint_positions}')
        # print(f'tgt joint vel: {action.joint_velocities}')
        # print(f'tgt joint pos len: {len(action.joint_positions)}')
        # print(f'now joint pos: {self._articulation.get_joint_positions()}')
        # print(f'now joint pos len: {len(self._articulation.get_joint_positions())}')
        # print(f'tgt joint pos: {action}')
        # print(f'tgt joint pos: {action.joint_indices}')
        self._articulation.apply_action(action)
        
        print(f'gains: {self._articulation.get_articulation_controller().get_gains()[0]}')
        
        # self._articulation.set_joint_velocities(
        #     velocities=action.joint_velocities,
        #     # joint_indices=np.array([7, 10, 13, 15, 17, 19, 21]),
        #     # joint_indices=np.arange(27)[-18:-11],
        #     # joint_indices=np.array([1, 4, 7, 10, 13, 16, 19]),
        #     joint_indices=self._right_arm_indices,
        # )
        
        # self._articulation.set_joint_positions(
        #     positions=action.joint_positions,
        #     # joint_indices=np.array([7, 10, 13, 15, 17, 19, 21]),
        #     # joint_indices=np.arange(27)[-18:-11],
        #     joint_indices=np.array([1, 4, 7, 10, 13, 16, 19]),
        #     # joint_indices=np.array([1, 4, 7, 10, 13, 16, 19]) + 1,
        # )
        
        self._my_world.step()
        
        # print(f'now joint pos: {self._articulation.get_joint_positions(joint_indices=self._left_arm_indices)}')
        # print(f'now joint vel: {self._articulation.get_joint_velocities(joint_indices=self._left_arm_indices)}')

    def reset(self):
        # Rmpflow is stateless unless it is explicitly told not to be
        if self._dbg_mode:
            # RMPflow was set to roll out robot state internally, assuming that all returned joint targets were hit exactly.
            self._rmpflow.reset()
            if self._visual_out:
                self._rmpflow.visualize_collision_spheres()

        self._my_world.reset()

        for _ in range(120):
            self._my_world.step()



# add noise to the camera
# /home/mad0g4/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.ros_bridge/camera_noise.py
# learn to get perspective camera prim