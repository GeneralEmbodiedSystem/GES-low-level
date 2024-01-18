from omnigibson.robots import TiagoAdvanced
from omnigibson.utils.control_utils import IKSolver


def get_ik_solver(robot: TiagoAdvanced):
    """Get ik solver for a robot

    Args:
        robot (TiagoAdvanced): the robot to get ik solver

    Returns:
        IKSolver: the ik solver for the robot
    """
    ik_solver = IKSolver(
        robot_description_path=robot.robot_arm_descriptor_yamls[robot.default_arm],
        robot_urdf_path=robot.urdf_path,
        default_joint_pos=robot.get_joint_positions()[robot.ik_control_idx],
        eef_name=robot.eef_link_names[robot.default_arm],
    )
    return ik_solver

def get_rmpflow_solver(robot: TiagoAdvanced):
    """Get rmpflow solver for a robot

    Args:
        robot (TiagoAdvanced): the robot to get rmpflow solver

    
    Returns:
        RMPflowSolver: the rmpflow solver for the robot  
    """
    rmpflow_solver = RMPflowSolver(
        robot_description_path=robot.robot_arm_descriptor_yamls[robot.default_arm],
        robot_urdf_path=robot.urdf_path,
        default_joint_pos=robot.get_joint_positions()[robot.ik_control_idx],
        eef_name=robot.eef_link_names[robot.default_arm],
    )
    return rmpflow_solver