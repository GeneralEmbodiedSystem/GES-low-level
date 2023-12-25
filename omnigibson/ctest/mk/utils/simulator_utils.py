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



def custom_step(robot: TiagoAdvanced, buffer: dict):
    tgt_joint_poses, tgt_control_idxes = [], []
    for k, v in buffer['tgt_joint_pos'].items():
        tgt_joint_poses.append(v[0])
        tgt_control_idxes.append(v[1])
    if len(tgt_joint_poses) > 0:
        tgt_joint_pos = np.concatenate(tgt_joint_poses)
        tgt_control_idx = np.concatenate(tgt_control_idxes)
        robot.set_joint_positions(
            tgt_joint_pos, 
            indices=tgt_control_idx,
            drive=True,
        )
    
    # simulator step
    og.sim.step()
