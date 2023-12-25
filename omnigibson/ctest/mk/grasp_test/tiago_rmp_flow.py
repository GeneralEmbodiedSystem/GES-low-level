import numpy as np
import os

from omni.isaac.kit import SimulationApp

headless = False
simulation_app = SimulationApp({"headless": headless, "anti_aliasing": 0})

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.objects import VisualCuboid

from omni.isaac.motion_generation import RmpFlow, ArticulationMotionPolicy

from examples.tiago_rmpflow_example import TiagoRmpFlowExample



# def set_perspective_to_camera(position: tuple, orientation: tuple):
#     from omni.isaac.core.utils.render_product import set_camera_prim_path
#     from omni.kit.viewport.utility import get_active_viewport

#     from pxr import Gf, UsdGeom, Usd

#     CAMERA_STAGE_PATH = "/Camera"

#     # Creating a Camera prim
#     camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
#     xform_api = UsdGeom.XformCommonAPI(camera_prim)
#     xform_api.SetTranslate(Gf.Vec3d(*position))
#     xform_api.SetRotate(orientation, UsdGeom.XformCommonAPI.RotationOrderXYZ)
#     camera_prim.GetHorizontalApertureAttr().Set(21)
#     camera_prim.GetVerticalApertureAttr().Set(16)
#     camera_prim.GetProjectionAttr().Set("perspective")
#     camera_prim.GetFocalLengthAttr().Set(24)
#     camera_prim.GetFocusDistanceAttr().Set(400)

#     simulation_app.update()

#     # grab our render product and directly set the camera prim
#     render_product_path = get_active_viewport().get_render_product_path()
#     set_camera_prim_path(render_product_path, CAMERA_STAGE_PATH)

# set_perspective_to_camera(
#     position=[2.0, -1.0, 0.7],
#     orientation=[87.7, 0.0, 65.4],
# )

tiago_rmp_example = TiagoRmpFlowExample()
tiago_rmp_example.load_example_assets()
tiago_rmp_example.setup()
tiago_rmp_example.reset()

while(True):
    tiago_rmp_example.update()
    # tiago_rmp_example._my_world.step()


