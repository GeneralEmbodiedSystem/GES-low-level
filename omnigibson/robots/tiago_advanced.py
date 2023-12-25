from typing import Dict, Optional, Union
import os

import numpy as np

from omnigibson.macros import gm
import omnigibson.utils.transform_utils as T
from omnigibson.macros import create_module_macros
from omnigibson.robots.active_camera_robot import ActiveCameraRobot
from omnigibson.robots.manipulation_robot import GraspingPoint, ManipulationRobot
from omnigibson.robots.locomotion_robot import LocomotionRobot
from omnigibson.robots.tiago import Tiago
from omnigibson.utils.python_utils import assert_valid_key
from omnigibson.utils.usd_utils import JointType
from omnigibson.sensors import VisionSensor, ScanSensor, create_sensor
import omnigibson.utils.transform_utils as T

from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.usd

from pxr import Gf

def calc_tilt_pan_from_direction(direction: np.ndarray):
    '''
    @param direction: np.ndarray([x, y, z])
    @return tilt_angle, pan_angle: tilt(vertical), pan(horizontal)
    '''
    # tilting: vertical, panning: horizontal
    direction = direction / np.linalg.norm(direction)
    # xy_direction = direction[:2] / np.linalg.norm(direction[:2])
    # xy_direction = direction[:2]
    z = direction[2]
    pan_angle = np.arctan2(direction[1], direction[0])
    tilt_angle = np.arcsin(z)
    return tilt_angle, pan_angle

# def calc_orientation_from_tilt_pan(tilt_angle, pan_angle):
#     quat = T.euler2quat([
#         0.0,
#         tilt_angle,
#         np.pi / 2 - pan_angle]
#     )
#     return quat

# def calc_orientation_from_direction(direction: np.ndarray):
#     # tilt_angle, pan_angle = calc_tilt_pan_from_direction(direction)
#     # quat = calc_orientation_from_tilt_pan(tilt_angle, pan_angle)
#     quat = np.concatenate((direction, np.array([0.])))
#     return quat

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

class TiagoAdvanced(Tiago):
    def _post_load(self):
        super()._post_load()
        # self._eyes = get_prim_at_path(f"{self._prim_path}/eyes")
        self.focal_length = 10.0 # zoom out
        # Whether the robot is ready to perform actions
        self.ready = True
        # Add more modalities
        self.vision_sensor.add_modality("depth_linear")
    
    def look_at(self, goal_pos: np.ndarray):
        '''
        @param goal_pos: world position of goal to fetch
        @return camera_joint_position, camera_control_idx: ...
        '''
        # camera pos wrong, should be eyes
        # camera_position, camera_orientation = self.vision_sensor.get_position_orientation()
        camera_position = get_pos_by_prim_path(self.eyes_prim_path)
        # print(f"camera_position: {camera_position}, {type(camera_position)}")
        # print(f"goal_pos: {goal_pos}")
        # print(f"direction: {goal_pos - camera_position}")
        direction = goal_pos - camera_position
        tilt, pan = calc_tilt_pan_from_direction(direction)
        # camera_joint_position = np.array([tilt, pan])
        camera_joint_position = np.array([pan, tilt])
        # print(f"camera_joint_position: {camera_joint_position}")
        # TODO: convert direction to orientation, set camera orientation
        # tgt_camera_orientation = calc_orientation_from_direction(direction)
        # tgt_camera_orientation = calc_orientation_from_tilt_pan(pan, tilt)
        # print(f"tgt_camera_orientation: {tgt_camera_orientation}")
        # self.vision_sensor.set_orientation(tgt_camera_orientation)
        
        return camera_joint_position, self.camera_control_idx
        # self.set_joint_positions(
        #     positions=camera_joint_position,
        #     indices=self.camera_control_idx,
        #     drive=True,
        # )
    
    # https://forums.developer.nvidia.com/t/the-camera-crops-objects-when-i-zoom-in/220278/8
    # def avoid_crop_when_zoom_in(self):
    #     pass
    
    def get_image(self) -> Dict:
        '''
        @return (dict): TiagoAdvanced 上的摄像机的图像{
            'rgb': np.ndarray,
            'depth': np.ndarray,
        }
        '''
        cam = self.vision_sensor
        vision_res = cam.get_obs()
        return vision_res
    
    @property
    def focal_length(self):
        return self.vision_sensor.focal_length

    @focal_length.setter
    def focal_length(self, value):
        self.vision_sensor.focal_length = value
    
    @property
    def clipping_range(self):
        return self.vision_sensor.clipping_range

    @clipping_range.setter
    def clipping_range(self, value):
        self.vision_sensor.clipping_range = value
    
    @property
    def vision_sensor(self):
        '''
        @return (VisionSensor, None): FetchAdvanced 上的摄像机的传感器
        '''
        required_sensor = None
        for sensor_name, sensor in self.sensors.items():
            if isinstance(sensor, VisionSensor):
                required_sensor = sensor
                break
        return required_sensor
    
    # @property
    # def eyes(self):
    #     return self._eyes

    @property
    def eyes_prim_path(self):
        '''
        @return (str): e.g. /world/robot/eyes
        '''
        return f"{self._prim_path}/eyes"


    # @property
    # def _default_controllers(self):
    #     # Always call super first
    #     controllers = super()._default_controllers

    #     # We use multi finger gripper, differential drive, and IK controllers as default
    #     controllers["base"] = "DifferentialDriveController"
    #     controllers["camera"] = "JointController"
    #     for arm in self.arm_names:
    #         controllers["arm_{}".format(arm)] = "InverseKinematicsController"
    #         controllers["gripper_{}".format(arm)] = "MultiFingerGripperController"
    #     return controllers

    # @property
    # def _default_base_controller_configs(self):
    #     dic = {
    #         "name": "DifferentialDriveController",
    #         "control_freq": self._control_freq,
    #         "control_limits": self.control_limits,
    #         "use_delta_commands": False,
    #         "motor_type": "velocity",
    #         "compute_delta_in_quat_space": [(3, 4, 5)],
    #         "dof_idx": self.base_control_idx,
    #     }
    #     return dic

    '''
    Fetch inherit TwoWheelRobot, so base controller is DifferentialDriveController
    Tiago inherit LocomotionRobot, so base controller is JointController
    '''

    # @property
    # def _default_controller_config(self):
    #     # Grab defaults from super method first
    #     cfg = super()._default_controller_config

    #     # Get default base controller for omnidirectional Tiago
    #     cfg["base"] = {"DifferentialDriveController": self._default_base_controller_configs}

    #     for arm in self.arm_names:
    #         for arm_cfg in cfg["arm_{}".format(arm)].values():

    #             if arm == "left":
    #                 # Need to override joint idx being controlled to include trunk in default arm controller configs
    #                 arm_cfg["dof_idx"] = np.concatenate([self.trunk_control_idx, self.arm_control_idx[arm]])

    #             # If using rigid trunk, we also clamp its limits
    #             # TODO: How to handle for right arm which has a fixed trunk internally even though the trunk is moving
    #             # via the left arm??
    #             if self.rigid_trunk:
    #                 arm_cfg["control_limits"]["position"][0][self.trunk_control_idx] = \
    #                     self.untucked_default_joint_pos[self.trunk_control_idx]
    #                 arm_cfg["control_limits"]["position"][1][self.trunk_control_idx] = \
    #                     self.untucked_default_joint_pos[self.trunk_control_idx]

    #     return cfg
    
    @property
    def ik_control_idx(self):
        return np.concatenate([self.trunk_control_idx, self.arm_control_idx[self.default_arm]])
    
    @property
    def tucked_default_joint_pos(self):
        pos = np.zeros(self.n_dof)
        # Keep the current joint positions for the base joints
        pos[self.base_idx] = self.get_joint_positions()[self.base_idx]
        pos[self.trunk_control_idx] = 0
        pos[self.camera_control_idx] = np.array([0.0, 0.0])
        for arm in self.arm_names:
            pos[self.gripper_control_idx[arm]] = np.array([0.045, 0.045])  # open gripper
            pos[self.arm_control_idx[arm]] = np.array(
                # [-1.10, 1.47, 2.71, 1.71, -1.57, 1.39, 0]
                [-0.7173956, 1.570797, 2.4387307, 2.1595283, 0.56682402, 0.35669315, 1.3414247]
            )
        return pos

    @property
    def camera_control_idx(self):
        """
        Returns:
            n-array: Indices in low-level control vector corresponding to [pan, tilt] camera joints.
        """
        # actually it's [pan, tilt]
        return np.array([9, 12])

    @property
    def front_right_scan_sensor(self):
        '''
        @return (VisionSensor, None): the Lidar sensor on the front right of TiagoAdvanced
        '''
        if not hasattr(self, 'front_right_lidar'):
            required_sensor = None
            for sensor_name, sensor in self.sensors.items():
                if isinstance(sensor, ScanSensor) and \
                    'front' in sensor_name:
                    required_sensor = sensor
                    break
            setattr(self, 'front_right_lidar', required_sensor)
        return getattr(self, 'front_right_lidar')
    
    @property
    def rear_left_scan_sensor(self):
        '''
        @return (VisionSensor, None): the Lidar sensor on the rear left of TiagoAdvanced
        '''
        if not hasattr(self, 'rear_left_lidar'):
            required_sensor = None
            for sensor_name, sensor in self.sensors.items():
                if isinstance(sensor, ScanSensor) and \
                    'rear' in sensor_name:
                    required_sensor = sensor
                    break
            setattr(self, 'rear_left_lidar', required_sensor)
        return getattr(self, 'rear_left_lidar')
    
    @property
    def front_left_scan_sensor(self):
        '''
        @return (VisionSensor, None): the Lidar sensor on the front left of TiagoAdvanced
        '''
        if not hasattr(self, 'front_left_lidar'):
            raise ValueError("`front_left_lidar` has not been initialized. Please call `init_scan_sensor` first.")
        return getattr(self, 'front_left_lidar')
    
    @property
    def rear_right_scan_sensor(self):
        '''
        @return (VisionSensor, None): the Lidar sensor on the rear right of TiagoAdvanced
        '''
        if not hasattr(self, 'rear_right_lidar'):
            raise ValueError("`rear_right_lidar` has not been initialized. Please call `init_scan_sensor` first.")
        return getattr(self, 'rear_right_lidar')
    
    def init_scan_sensor(self):
        '''
        @return (None): init all the Lidar sensors for TiagoAdvanced
        '''
        front_right_pos = self.front_right_scan_sensor.get_position()
        rear_left_pos = self.rear_left_scan_sensor.get_position()
        front_left_pos = front_right_pos.copy()
        front_left_pos[1] = rear_left_pos[1]
        rear_right_pos = rear_left_pos.copy()
        rear_right_pos[1] = front_right_pos[1]

        # Set front right sensor
        self.front_right_scan_sensor.set_position(
            front_right_pos + np.array([0.05, -0.05, 0])
        )
        self.front_right_scan_sensor.horizontal_fov = 170.0

        # Set rear left sensor
        self.rear_left_scan_sensor.set_position(
            rear_left_pos + np.array([-0.05, 0.05, 0])
        )
        self.rear_left_scan_sensor.horizontal_fov = 170.0

        # Get the prim path of the robot self
        robot_prim_path = self._prim_path
        print(robot_prim_path)
        # Create an additional sensor: front left
        lidar_front_left = create_sensor(
            sensor_type="Lidar",
            prim_path=robot_prim_path+"/base_front_laser_link/Lidar_left",
            name="Lidar_left",
            sensor_kwargs={
                "horizontal_fov": 170.0,
            }
        )
        lidar_front_left.load()
        lidar_front_left._load()
        lidar_front_left._post_load()
        lidar_front_left._initialize()
        lidar_front_left.load_observation_space()
        lidar_front_left._load_observation_space()
        lidar_front_left.set_position(
            front_left_pos + np.array([0.05, 0.05, 0])
        )
        lidar_front_left.set_orientation(
            np.array([0.92388, 0.38268, 0.0, 0])
        )
        setattr(self, 'front_left_lidar', lidar_front_left)

        # Create an additional sensor: rear right
        lidar_rear_right = create_sensor(
            sensor_type="Lidar",
            prim_path=robot_prim_path+"/base_rear_laser_link/Lidar_right",
            name="Lidar_right",
            sensor_kwargs={
                "horizontal_fov": 170.0,
            }
        )
        lidar_rear_right.load()
        lidar_rear_right._load()
        lidar_rear_right._post_load()
        lidar_rear_right._initialize()
        lidar_rear_right.load_observation_space()
        lidar_rear_right._load_observation_space()
        lidar_rear_right.set_position(
            rear_right_pos + np.array([-0.05, -0.05, 0])
        )
        lidar_rear_right.set_orientation( 
            np.array([0.38268, -0.92388, 0.0, 0.0])
        )
        setattr(self, 'rear_right_lidar', lidar_rear_right)

    def get_scan(self) -> Dict:
        '''
        @return (dict): the Lidar data in TiagoAdvanced{
            'scan': np.ndarray,
            'scan_noise': np.ndarray, # May not exist
        }
        '''
        scan_sensors = [
            self.front_right_scan_sensor, self.rear_left_scan_sensor,
            self.front_left_scan_sensor, self.rear_right_scan_sensor,
        ]
        scan_res = {}
        for sensor in scan_sensors:
            scan_res[sensor.name] = sensor.get_obs()
        
        return scan_res

