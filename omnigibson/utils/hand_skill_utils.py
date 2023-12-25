import numpy as np

from omnigibson.robots import TiagoAdvanced


def check_joint_move(robot: TiagoAdvanced, buffer: dict, control_idx, eps=1e-6):
    '''
    check end of a release according to the joint pos delta
    
    Args:
        robot: a instance of TiagoAdvanced.
        buffer: a dict serve as a buffer.
        control_idx: control index of the joint of the robot.
    '''
    last_joint_pos = buffer.get('last_joint_pos', None)
    now_joint_pos = robot.get_joint_positions()[control_idx]
    buffer['last_joint_pos'] = now_joint_pos
    end_ik_flag = False
    if last_joint_pos is not None:
        delta_joint_pos = now_joint_pos - last_joint_pos
        end_ik_flag = (np.sum(np.abs(delta_joint_pos)) < eps)
    
    return end_ik_flag