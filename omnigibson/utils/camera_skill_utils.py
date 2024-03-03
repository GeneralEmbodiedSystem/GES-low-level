import numpy as np

from omnigibson.robots import TiagoAdvanced

def convert_degree_to_radian(degree: float):
    return degree / 180 * np.pi

def execute_look_up(robot: TiagoAdvanced, buffer: dict, degree: float):
    # assert degree >= 0 and degree <= 60
    tilt = convert_degree_to_radian(degree)
    if 'tgt_joint_pos' not in buffer:
        buffer['tgt_joint_pos'] = {}
    buffer['tgt_joint_pos']['camera'] = (
        np.array([0., tilt]),
        robot.camera_control_idx
    )

def execute_look_down(robot: TiagoAdvanced, buffer: dict, degree: float):
    # assert degree >= 0 and degree <= 60
    tilt = convert_degree_to_radian(degree)
    if 'tgt_joint_pos' not in buffer:
        buffer['tgt_joint_pos'] = {}
    buffer['tgt_joint_pos']['camera'] = (
        np.array([0., -tilt]),
        robot.camera_control_idx
    )

def execute_look_forward(robot: TiagoAdvanced, buffer: dict):
    if 'tgt_joint_pos' not in buffer:
        buffer['tgt_joint_pos'] = {}
    buffer['tgt_joint_pos']['camera'] = (
        np.array([0., 0.]),
        robot.camera_control_idx
    )