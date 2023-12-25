import numpy as np

from omnigibson.utils.transform_utils import quat2euler


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