from omnigibson.custom_env.exceptions import BaseRobotException


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
        self.recover_steps = 10
        self.cur_recover_step = 0
    
    def __str__(self):
        return "Collision detected: robot cannot move {}. It has stopped now.".format(self.direction)

    def get_recovery_action(self, action):
        if self.direction == "forward":
            action[0] = -0.06
        elif self.direction == "backward":
            action[0] = 0.06
        elif self.direction == "left":
            action[1] = -0.06
        elif self.direction == "right":
            action[1] = 0.06
        
        return action
