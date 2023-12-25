import numpy as np

from omnigibson.robots import TiagoAdvanced


def check_joint_move(robot: TiagoAdvanced, single_reach_scratch: dict, control_idx, eps=1e-10):
    '''
    check end of a release according to the joint pos delta
    
    Args:
        robot: a instance of TiagoAdvanced.
        single_reach_scratch: a dict serve as a scratch.
        control_idx: control index of the joint of the robot.
    '''
    last_joint_pos = single_reach_scratch.get('last_joint_pos', None)
    now_joint_pos = robot.get_joint_positions()[control_idx]
    single_reach_scratch['last_joint_pos'] = now_joint_pos
    end_ik_flag = False
    if last_joint_pos is not None:
        delta_joint_pos = now_joint_pos - last_joint_pos
        end_ik_flag = (np.sum(np.abs(delta_joint_pos)) < eps)
    
    return end_ik_flag