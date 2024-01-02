"""
Utilities for checking whether the robot collides with the environment and
handling the collision.
"""
from typing import List

import numpy as np

from omnigibson.object_states import ContactBodies
from omnigibson.custom_env.exceptions import MoveException
from omnigibson.utils.transform_utils import quat2euler
from omnigibson.robots.tiago_advanced import TiagoAdvanced

from omnigibson.custom_env.utils.eular_utils import check_walk_straight


# Lidar is DISABLED temporarily
# because the orientation of lidar's axis
# is absolute, not aware of robot's orientation
def detect_collision_by_lidar(global_states) -> bool:
    """
    Detect whether the robot collides with the environment by the Lidar point clouds.

    Args:
        global_states: a dict used to store global variables, which are shared with
            other functions.

    Returns:
        True if the robot collides with the environment, False otherwise.
    """
    EPSILON = 2e-2  # Threshold for collision detection

    move_action = global_states['active_actions']['move_action']
    robot = global_states['robot']

    # Get positions of the front right and rear left sensors
    front_right_pos = robot.front_right_scan_sensor.get_position()
    rear_left_pos = robot.rear_left_scan_sensor.get_position()

    x_len, y_len = np.abs(front_right_pos - rear_left_pos)[:2]

    if move_action['func'] == 'move_forward':
        lidar = robot.front_right_scan_sensor
        point_cloud = robot.get_scan_point_cloud(lidar, rotate=-np.pi/4)

        within_front_bounds = (point_cloud[:, 0] >= -x_len) & \
            (point_cloud[:, 0] <= 0) & \
            (point_cloud[:, 1] >= 0) & \
            (point_cloud[:, 1] <= EPSILON)
        return np.any(within_front_bounds)
    
    elif move_action['func'] == 'move_right':
        lidar = robot.front_right_scan_sensor
        point_cloud = robot.get_scan_point_cloud(lidar, rotate=-np.pi/4)

        within_right_bounds = (point_cloud[:, 0] >= 0) & \
            (point_cloud[:, 0] <= EPSILON) & \
            (point_cloud[:, 1] >= -y_len) & \
            (point_cloud[:, 1] <= 0)
        return np.any(within_right_bounds)

    elif move_action['func'] == 'move_backward':
        lidar = robot.rear_left_scan_sensor
        point_cloud = robot.get_scan_point_cloud(lidar, rotate=3*np.pi/4)

        within_rear_bounds = (point_cloud[:, 0] >= 0) & \
            (point_cloud[:, 0] <= x_len) & \
            (point_cloud[:, 1] >= -EPSILON) & \
            (point_cloud[:, 1] <= 0)
        return np.any(within_rear_bounds)
    
    elif move_action['func'] == 'move_left':
        lidar = robot.rear_left_scan_sensor
        point_cloud = robot.get_scan_point_cloud(lidar, rotate=3*np.pi/4)

        within_left_bounds = (point_cloud[:, 0] >= -EPSILON) & \
            (point_cloud[:, 0] <= 0) & \
            (point_cloud[:, 1] >= 0) & \
            (point_cloud[:, 1] <= y_len)
        return np.any(within_left_bounds)

    else:
        return False


# Note: This function is DEPRECATED since 
# the collision will unexpectedly occur when grasping.
def detect_collision_by_contact_bodies(robot: object, ignore_objs: tuple) -> bool:
    """
    Detect whether the robot collides with the environment by the ContactBodies state.

    Args:
        robot: the robot object.
        ignore_objs: the objects to be ignored when checking the collision.

    Returns:
        True if the robot collides with the environment, False otherwise.
    """
    return len(robot.states[ContactBodies].get_value(ignore_objs=ignore_objs)) > 0


# Robot's normal speed: 3.1~3.2e-3
def detect_collision_by_pos_delta(robot: TiagoAdvanced, last_pos, eps=3e-3):
    '''
    check if robot move according to the pos delta. If the robot moves significantly
    slower than the expected speed, we consider it as a collision.
    
    Returns:
        collision_flag (bool): True if the robot collision.
    '''
    now_pos = robot.get_position()
    delta_pos = np.linalg.norm(np.array(now_pos) - np.array(last_pos))
    return delta_pos < eps


def handle_collision(global_states: dict) -> None:
    """
    Check whether the robot collides with the environment and handle the collision.

    Args:
        global_states: a dict used to store global variables, which are shared with
            other functions.
    """

    move_action = global_states['active_actions']['move_action']
    move_exception = global_states['active_actions']['move_exception']
    if move_action is None or move_exception is not None:
        return
    
    # TODO: Add collision detection for turn_left and turn_right
    if move_action['func'] not in ['move_forward', 'move_backward', 
            'move_left', 'move_right']:
        # Restore collision_vars
        global_states['collision_vars'] = {
            'last_position': None,
            # 'init_orientation_z': None,
        }
        return
    
    # extra the params
    robot = global_states['robot']
    robot_exception = MoveException(move_action['func'][5:])
    collision_vars = global_states['collision_vars']
    last_position = collision_vars.get('last_position', None)
    # init_orientation_z = collision_vars.get('init_orientation_z', None)
    
    # check_walk_straight is not used because of unknown bugs
    # check collision if not at the first step
    if last_position is not None and \
            detect_collision_by_pos_delta(robot, last_position):
        print("COLLISION!")
        # If the robot collides with the environment, raise an exception
        global_states['active_actions']['move_exception'] = robot_exception
        # The robot will stop moving
        global_states['active_actions']['move_action'] = None 
        # The exception will exist until recovery
        # Make the robot not ready
        robot.ready = False
        return

    # update collision vars
    collision_vars['last_position'] = robot.get_position()
    # collision_vars['init_orientation_z'] = quat2euler(robot.get_orientation())[2]
    
    # Lidar is DISABLED temporarily
    # because the orientation of lidar's axis
    # is absolute, not aware of robot's orientation
    # if detect_collision_by_lidar(global_states):
    #     print("COLLISION!")
    #     global_states['active_actions']['move_exception'] = robot_exception
    #     global_states['active_actions']['move_action'] = None 
    #     robot.ready = False
