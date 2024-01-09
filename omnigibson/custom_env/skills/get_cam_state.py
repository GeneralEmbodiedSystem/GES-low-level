from typing import Union

from servers import PipeManager
from omnigibson.robots import FetchAdvanced, TiagoAdvanced


def emit_get_cam_state(pipe: PipeManager, global_states: dict):
    """
    Get the state of the egocentric camera for robot

    cam_state: {
        'pose': list[float] of shape (4, 4),
        'fov': float,
        'focal_length': float,
        'horizontal_aperture': float,
        'view_projection_matrix': list[float] of shape (4, 4),
        'resolution': {
            'width': int,
            'height': int
        },
        'clipping_range': list[float] of shape (2, ),
    }
    """
    robot: Union[FetchAdvanced, TiagoAdvanced] = global_states['robot']
    obs_res = robot.get_cam_obs()
    
    cam_state = obs_res['camera']
    cam_state['pose'] = cam_state['pose'].tolist()
    cam_state['view_projection_matrix'] = cam_state['view_projection_matrix'].tolist()
    cam_state['clipping_range'] = list(cam_state['clipping_range'])
    
    pipe.write(
        data=cam_state,
        role="server"
    )
