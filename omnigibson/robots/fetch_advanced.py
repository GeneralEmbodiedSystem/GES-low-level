from typing import Dict

import os
import numpy as np

from omnigibson.robots.fetch import Fetch
from omnigibson.sensors import VisionSensor

class FetchAdvanced(Fetch):
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

    
