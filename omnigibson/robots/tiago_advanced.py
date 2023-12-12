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
from omnigibson.sensors import VisionSensor
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
        @return (dict): FetchAdvanced 上的摄像机的图像{
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



    
    