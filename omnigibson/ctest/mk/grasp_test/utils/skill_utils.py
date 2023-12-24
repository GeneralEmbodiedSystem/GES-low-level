from typing import Union, Dict, List, Any

import pprint
import cv2

import numpy as np

import omnigibson as og
from omnigibson.macros import gm
from omnigibson.sensors import VisionSensor
from omnigibson.utils.ui_utils import choose_from_options, KeyboardRobotController
from omnigibson.utils.asset_utils import get_available_g_scenes, get_available_og_scenes
from omnigibson.robots import FetchAdvanced

from omnigibson.robots.turtlebot import Turtlebot
from omnigibson.robots.tiago_advanced import TiagoAdvanced
from omnigibson.scenes.interactive_traversable_scene import InteractiveTraversableScene
from omnigibson.utils.asset_utils import get_og_assets_version
from omnigibson.utils.control_utils import IKSolver
from omnigibson.objects import PrimitiveObject, DatasetObject, LightObject

import carb
import omni

from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.physx import get_physx_interface
import omni.usd

from pxr import Gf

from utils.prim_utils import get_pos_by_prim_path, get_pos_by_prim_path_physxApi



def lift_trunk_body(robot: TiagoAdvanced):
    robot.set_joint_positions(
        np.array([1.]),
        indices=robot.trunk_control_idx,
        # drive=True,
    )

# Transfer rgba image to rgb image
def rgba2rgb(rgba, background=(255,255,255)):
    row, col, ch = rgba.shape
    if ch != 4:
        raise ValueError('RGBA image should have 4 channels.')
    rgb = np.zeros((row, col, 3), dtype='float32')
    r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]
    a = np.asarray(a, dtype='float32') / 255.0
    R, G, B = background
    rgb[:,:,0] = r * a + (1.0 - a) * R
    rgb[:,:,1] = g * a + (1.0 - a) * G
    rgb[:,:,2] = b * a + (1.0 - a) * B

    return np.asarray(rgb, dtype='uint8')

def execute_record(vision_sensor: VisionSensor, buffer: dict, saved_vedio_path = None, fps = 60, end = False):
    '''
    record a frame when invoke with end=False
    empty cache & save vedio when end=True && saved_vedio_path is not None
    '''
    if 'record_frames' not in buffer:
        buffer['record_frames'] = list()
    record_frames = buffer['record_frames']
    
    sensor_obs = vision_sensor.get_obs()
    rgba_image = sensor_obs['rgb']
    rgb_image = rgba2rgb(rgba_image)
    
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    
    # cv2.imshow('Image', bgr_image)
    # # 等待按键后关闭窗口
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    record_frames.append(bgr_image)
    
    if end and len(record_frames) > 0:
        height, width, layers = record_frames[0].shape
        size = (width, height)
        out = cv2.VideoWriter(saved_vedio_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        for frame in record_frames:
            out.write(frame)
        out.release()
        
        buffer.pop('record_frames')

# Define a helper function for executing specific end-effector commands using the ik solver
def execute_ik(ik_solver: IKSolver, robot: TiagoAdvanced, pos, buffer: dict, quat=None, max_iter=100):
    control_idx = np.concatenate([robot.trunk_control_idx, robot.arm_control_idx[robot.default_arm]])
    joint_pos = ik_solver.solve(
        # target_pos=pos, # local target quaternion orientation in robot's base coordinate frame
        # target_pos=pos - get_pos_by_prim_path_physxApi("/World/robot/base_link"), # local target quaternion orientation in robot's base coordinate frame
        target_pos=pos - robot.get_position(), # local target quaternion orientation in robot's base coordinate frame
        # target_quat=quat if quat is None else quat - robot.get_orientation(),
        target_quat=quat,
        max_iterations=max_iter,
        initial_joint_pos=robot.get_joint_positions()[control_idx],
        tolerance_pos = 0.00001
    )
    if joint_pos is not None:
        # og.log.info("Solution found. Setting new arm configuration.")
        pass
    else:
        # og.log.info("EE position not reachable.")
        pass

    if 'last_ik_joint_position' in buffer:
        same_idx = np.array([5, 7])
        joint_pos[same_idx] = buffer['last_ik_joint_position'][same_idx]
    print(f'ik joint_pos: {joint_pos}')

    # set tgt position
    # robot.set_joint_positions(joint_pos, indices=control_idx, drive=True)
    buffer['tgt_joint_pos']['ik'] = (joint_pos, control_idx)
    
    buffer['last_ik_joint_position'] = joint_pos

def execute_look_at(robot: TiagoAdvanced, look_at_pos, buffer: dict):
    camera_joint_pos, camera_idx = robot.look_at(goal_pos=look_at_pos)
    buffer['tgt_joint_pos']['camera'] = (camera_joint_pos, camera_idx)

def execute_grasp(robot: TiagoAdvanced, buffer: dict, left_or_right: str = 'left'):
    buffer['tgt_joint_pos']['grasp'] = ([0., 0.], robot.gripper_control_idx[left_or_right])

def execute_release(robot: TiagoAdvanced, buffer: dict, left_or_right: str = 'left'):
    buffer['tgt_joint_pos']['release'] = ([0.045, 0.045], robot.gripper_control_idx[left_or_right])
