from omni.isaac.kit import SimulationApp

headless = False
simulation_app = SimulationApp({"headless": headless, "anti_aliasing": 0})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.prims import get_prim_at_path, define_prim

import carb

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()

tiago_usd_path = '/home/mad0g4/Desktop/code/GES-low-level/omnigibson/data/assets/models/tiago/tiago_dual_omnidirectional_stanford/tiago_dual_omnidirectional_stanford.usd'

def attach_usd_to_prim(prim_path: str, usd_path: str):
    prim = get_prim_at_path(prim_path)
    if not prim.IsValid():
        prim = define_prim(prim_path, "Xform")
        if usd_path:
            prim.GetReferences().AddReference(usd_path)
        else:
            carb.log_error("no valid usd path defined to create new robot")

# prim_path = '/World/jetbot'
prim_path = '/jetbot'
robot_name = 'jetbot'
attach_usd_to_prim(prim_path, tiago_usd_path)

robot = my_world.scene.add(
    Robot(
        prim_path=prim_path,
        name=robot_name,
    )
)


while True:
    my_world.step(render=True)