class BaseRobotException:
    """
    Base exception class for all exceptions in the robotic simulation.
    This is different from the built-in Exception class in Python
    """

    def __init__(self):
        # Whether the robot has recovered from the exception
        self.recovered = False
