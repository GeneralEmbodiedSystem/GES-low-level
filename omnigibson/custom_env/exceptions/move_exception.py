import numpy as np

from omnigibson.custom_env.exceptions import BaseRobotException
from omnigibson.utils.transform_utils import quat2euler
from omnigibson.custom_env.utils.constant_utils import MOVE_FORWARD_VEL, MOVE_BACKWARD_VEL, MOVE_LEFT_RIGHT_VEL


class MoveException(BaseRobotException):
    """
    Exception class for robot movement when
    collision is detected.
    """

    def __init__(self, direction):
        super().__init__()
        # The direction of the movement before the exception
        self.direction = direction
        # Number of steps need to be taken to recover from the exception
        self.recover_steps = 20 # About 3e-3 x 30 = 0.06 meters
        self.cur_recover_step = 0
    
    def __str__(self):
        return "Robot cannot move {} because of collision detected.".format(self.direction)

    def get_recovery_action(self, action, global_states: dict):
        robot = global_states['robot']
        z_radian = quat2euler(robot.get_orientation())[2]

        if self.direction == "forward":
            # Move backward to restore
            x_vel = MOVE_BACKWARD_VEL * np.cos(z_radian)
            y_vel = MOVE_BACKWARD_VEL * np.sin(z_radian)
        elif self.direction == "backward":
            # Move forward to restore
            x_vel = MOVE_FORWARD_VEL * np.cos(z_radian)
            y_vel = MOVE_FORWARD_VEL * np.sin(z_radian)
        elif self.direction == "left":
            # Move right to restore
            x_vel = MOVE_LEFT_RIGHT_VEL * np.sin(z_radian)
            y_vel = -MOVE_LEFT_RIGHT_VEL * np.cos(z_radian)
        elif self.direction == "right":
            # Move left to restore
            x_vel = -MOVE_LEFT_RIGHT_VEL * np.sin(z_radian)
            y_vel = MOVE_LEFT_RIGHT_VEL * np.cos(z_radian)
        
        action[0] = x_vel
        action[1] = y_vel

        return action
