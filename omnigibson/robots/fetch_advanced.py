from typing import Dict

import os
import numpy as np

from omnigibson.robots.fetch import Fetch
from omnigibson.sensors import VisionSensor


class FetchAdvanced(Fetch):
    # def __init__(
    #     self,
    #     # Shared kwargs in hierarchy
    #     name,
    #     prim_path=None,
    #     class_id=None,
    #     uuid=None,
    #     scale=None,
    #     visible=True,
    #     visual_only=False,
    #     self_collisions=False,
    #     load_config=None,
    #     fixed_base=False,

    #     # Unique to USDObject hierarchy
    #     abilities=None,

    #     # Unique to ControllableObject hierarchy
    #     control_freq=None,
    #     controller_config=None,
    #     action_type="continuous",
    #     action_normalize=True,
    #     reset_joint_pos=None,

    #     # Unique to BaseRobot
    #     obs_modalities="all",
    #     proprio_obs="default",

    #     # Unique to ManipulationRobot
    #     grasping_mode="physical",

    #     # # Unique to Fetch
    #     # rigid_trunk=False,
    #     # default_trunk_offset=0.365,
    #     # default_arm_pose="diagonal30",

    #     rigid_trunk=True,
    #     default_trunk_offset=0.165,
    #     default_arm_pose="diagonal30",

    #     **kwargs,
    # ):
    #     # kwargs['rigid_trunk'] = True
    #     # kwargs['default_trunk_offset'] = 0.165
    #     super().__init__(
    #         name,
    #         prim_path=prim_path,
    #         class_id=class_id,
    #         uuid=uuid,
    #         scale=scale,
    #         visible=visible,
    #         visual_only=visual_only,
    #         self_collisions=self_collisions,
    #         load_config=load_config,
    #         fixed_base=fixed_base,

    #         # Unique to USDObject hierarchy
    #         abilities=abilities,

    #         # Unique to ControllableObject hierarchy
    #         control_freq=control_freq,
    #         controller_config=controller_config,
    #         action_type=action_type,
    #         action_normalize=action_normalize,
    #         reset_joint_pos=reset_joint_pos,

    #         # Unique to BaseRobot
    #         obs_modalities=obs_modalities,
    #         proprio_obs=proprio_obs,

    #         # Unique to ManipulationRobot
    #         grasping_mode=grasping_mode,

    #         # # Unique to Fetch
    #         # rigid_trunk=False,
    #         # default_trunk_offset=0.365,
    #         # default_arm_pose="diagonal30",

    #         rigid_trunk=rigid_trunk,
    #         default_trunk_offset=default_trunk_offset,
    #         default_arm_pose=default_arm_pose,

    #         **kwargs,
    #     )

    def _post_load(self):
        super()._post_load()
        # self._eyes = get_prim_at_path(f"{self._prim_path}/eyes")
        self.focal_length = 10.0 # zoom out

    @property
    def focal_length(self):
        return self.vision_sensor.focal_length

    @focal_length.setter
    def focal_length(self, value):
        self.vision_sensor.focal_length = value
    
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
    
    def get_cam_obs(self) -> Dict:
        '''
        @return (dict): FetchAdvanced 上的摄像机的图像{
            'rgb': np.ndarray,
            'depth': np.ndarray,
        }
        '''
        cam = self.vision_sensor
        vision_res = cam.get_obs()
        return vision_res

    
