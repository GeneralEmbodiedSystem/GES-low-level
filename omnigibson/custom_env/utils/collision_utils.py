"""
Utilities for checking whether the robot collides with the environment and
handling the collision.
"""
from typing import List

import numpy as np

from omnigibson.object_states import ContactBodies
from omnigibson.custom_env.exceptions import MoveException


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


def handle_collision(global_states: dict, ignore_objs: tuple) -> None:
    """
    Check whether the robot collides with the environment and handle the collision.

    Args:
        global_states: a dict used to store global variables, which are shared with
            other functions.
        ignore_objs: the objects to be ignored when checking the collision.
            It will be IGNORED by Lidar.
    """

    move_action = global_states['active_actions']['move_action']
    move_exception = global_states['active_actions']['move_exception']
    if move_action is None or move_exception is not None:
        return
    
    if move_action['func'] not in ['move_forward', 'move_backward', 
        'move_left', 'move_right']:
        return
    
    robot = global_states['robot']
    robot_exception = MoveException(move_action['func'][5:])

    if detect_collision_by_contact_bodies(robot, ignore_objs):
        print("COLLISION!")
        # If the robot collides with the environment, raise an exception
        global_states['active_actions']['move_exception'] = robot_exception
        # The robot will stop moving
        global_states['active_actions']['move_action'] = None 
        # The exception will exist until recovery
        # Make the robot not ready
        robot.ready = False
        return

    lidar_depths = []
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
    
    if detect_collision_by_lidar(lidar_depths):
        print("COLLISION!")
        global_states['active_actions']['move_exception'] = robot_exception
        global_states['active_actions']['move_action'] = None 
        robot.ready = False
