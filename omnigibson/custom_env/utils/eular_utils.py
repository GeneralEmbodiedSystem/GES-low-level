import numpy as np

from omnigibson.utils.transform_utils import quat2euler

from omnigibson.robots import TiagoAdvanced


def check_turn_end(init_z_radian, left_or_right, turn_radian, now_z_radian):
    '''
    check the end condition of turn
    
    Args:
        init_z_radian (float): the initial z angle of the robot
            range: [-pi, pi)
        left_or_right (str): 'left' or 'right', 'left' is positive,
            'right' is negative
        turn_radian (float): the angle to turn, always positive
        now_z_radian (float): the current z angle of the robot
    '''
    sign = 1 if left_or_right == 'left' else -1
    delta_z_radian = turn_radian * sign
    end_z_radian = init_z_radian + delta_z_radian
    
    # out of range, and be modified
    modify_flag = ((sign * now_z_radian) < (sign * init_z_radian))
    if modify_flag:
        now_z_radian += 2 * np.pi * sign
    
    # in range, not end
    if (sign * now_z_radian) >= (sign * init_z_radian) and \
            (sign * now_z_radian) < (sign * end_z_radian):
        return False
    
    # out of range, end
    return True

def calculate_distance(start_pos, end_pos):
    '''
    calculate the distance between two points
    
    Args:
        start_pos (list): the start position of the robot
        end_pos (list): the end position of the robot
    '''
    return np.linalg.norm(np.array(start_pos) - np.array(end_pos))

def check_joint_still(robot: TiagoAdvanced, single_reach_scratch: dict, control_idx, eps=1e-4):
    '''
    check if joint move according to the joint pos delta
    
    Args:
        robot: a instance of TiagoAdvanced.
        single_reach_scratch: a dict serve as a scratch.
        control_idx: control index of the joint of the robot.
    
    Returns:
        end_ik_flag (bool): True if the joint keeps still.
    '''
    last_joint_pos = single_reach_scratch.get('last_joint_pos', None)
    now_joint_pos = robot.get_joint_positions()[control_idx]
    single_reach_scratch['last_joint_pos'] = now_joint_pos
    end_ik_flag = False
    if last_joint_pos is not None:
        delta_distance = calculate_distance(now_joint_pos, last_joint_pos)
        # print(f'operator: delta_distance: {delta_distance}')
        end_ik_flag = (delta_distance < eps)
    
    return end_ik_flag

def check_walk_straight(robot: TiagoAdvanced, init_orientation_z, eps=1e-1):
    '''
    check if robot walk straight according to the orientation_z
    
    Args:
        robot: a instance of TiagoAdvanced.
        init_orientation_z: orientation_z at first.
    
    Returns:
        walk_straight_flag (bool): True if the robot walks straight.
    '''
    now_orientation_z = quat2euler(robot.get_orientation())[2]
    return abs(now_orientation_z - init_orientation_z) < eps