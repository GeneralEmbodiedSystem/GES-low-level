"""
Utilities for checking whether the robot collides with the environment and
handling the collision.
"""
from typing import List

import numpy as np


def detect_collision_by_lidar(lidar_depths: List[np.ndarray]) -> bool:
    """
    Detect whether the robot collides with the environment by the Lidar depth results.

    Args:
        lidar_depth: the depth images obtained from the lidar, which are
            normalized to [0, 1].

    Returns:
        True if the robot collides with the environment, False otherwise.
    """
    EPSILON = 1e-5
    depth = np.concatenate(lidar_depths, axis=0)
    min_depth = np.min(depth)

    return min_depth < EPSILON


def handle_collision(local_vars: dict) -> None:
    """
    Check whether the robot collides with the environment and handle the collision.

    Args:
        local_vars: a dict used to store local variables, which are shared with
            other functions.
    """

    move_action = local_vars['active_actions']['move_action']
    if move_action is None:
        return
    
    lidar_depths = []
    robot = local_vars['robot']

    if move_action['func'] == 'move_forward':
        lidar_depths = [
            robot.front_right_scan_sensor.get_obs()['scan'],
            robot.front_left_scan_sensor.get_obs()['scan']
        ]
    elif move_action['func'] == 'move_backward':
        lidar_depths = [
            robot.rear_right_scan_sensor.get_obs()['scan'],
            robot.rear_left_scan_sensor.get_obs()['scan']
        ]
    elif move_action['func'] == 'move_left':
        lidar_depths = [
            robot.front_left_scan_sensor.get_obs()['scan'],
            robot.rear_left_scan_sensor.get_obs()['scan']
        ]
    elif move_action['func'] == 'move_right':
        lidar_depths = [
            robot.front_right_scan_sensor.get_obs()['scan'],
            robot.rear_right_scan_sensor.get_obs()['scan']
        ]
    
    if len(lidar_depths) > 0 and detect_collision_by_lidar(lidar_depths):
        print("COLLISION!")
        # If the robot collides with the environment, stop the robot
        local_vars['active_actions']['move_action'] = None
