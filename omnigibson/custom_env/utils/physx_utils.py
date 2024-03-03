import numpy as np


# https://forums.developer.nvidia.com/t/world-position-not-updating-during-simulation-run/245372/5
def get_pos_by_prim_path_physxApi(prim_path: str):
    '''
    @param prim_path: e.g. /world/robot/eyes/base_link
    @return position/translate: np.ndarray([x, y, z])
    '''
    from omni.physx import get_physx_interface
    from pxr import Gf
    
    curr_transform = get_physx_interface().get_rigidbody_transformation(prim_path)
    rv_success = curr_transform["ret_val"]
    if rv_success:
        curr_pos_f3 = curr_transform["position"]
        curr_pos = Gf.Vec3d(curr_pos_f3[0], curr_pos_f3[1], curr_pos_f3[2])
        curr_rot_f4 = curr_transform["rotation"]
        curr_rot_quat = Gf.Quatd(curr_rot_f4[3], curr_rot_f4[0], curr_rot_f4[1], curr_rot_f4[2])
        return np.array(curr_pos)
    return None