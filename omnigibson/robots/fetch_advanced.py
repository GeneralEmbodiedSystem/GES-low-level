from typing import Dict

import os
import numpy as np

from omnigibson.robots.fetch import Fetch
from omnigibson.sensors import VisionSensor

class FetchAdvanced(Fetch):
    
    @property
    def vision_sensor(self):
        required_sensor = None
        for sensor_name, sensor in self.sensors.items():
            if isinstance(sensor, VisionSensor):
                required_sensor = sensor
                break
        return required_sensor
    
    def get_image(self) -> Dict:
        cam = self.vision_sensor
        vision_res = cam.get_obs()
        return vision_res

    
