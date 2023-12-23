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
        self.recovered_steps = 10
    
    def __str__(self):
        return "Collision detected: robot cannot move {}. It has stopped now.".format(self.direction)

    def get_recovery_action(self):
        if self.direction == "forward":
            return {
                "func": "move_backward",
                "args": {
                    'vx': -0.06
                }
            }
        elif self.direction == "backward":
            return {
                "func": "move_forward",
                "args": {
                    'vx': 0.06
                }
            }
        elif self.direction == "left":
            return {
                "func": "move_right",
                "args": {
                    'vy': -0.06
                }
            }
        elif self.direction == "right":
            return {
                "func": "move_left",
                "args": {
                    'vy': 0.06
                }
            }
