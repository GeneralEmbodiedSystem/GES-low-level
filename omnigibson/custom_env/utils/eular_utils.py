import numpy as np

from omnigibson.utils.transform_utils import quat2euler


def check_turn_end(init_z_angle, left_or_right, turn_radian, now_z_angle):
    '''
    check the end condition of turn
    
    Args:
        init_z_angle (float): the initial z angle of the robot
            range: [-pi, pi)
        left_or_right (str): 'left' or 'right', 'left' is positive,
            'right' is negative
        turn_radian (float): the angle to turn, always positive
        now_z_angle (float): the current z angle of the robot
    '''
    sign = 1 if left_or_right == 'left' else -1
    delta_z_angle = turn_radian * sign
    end_z_angle = init_z_angle + delta_z_angle
    
    # out of range, and be modified
    modify_flag = ((sign * now_z_angle) < (sign * init_z_angle))
    if modify_flag:
        now_z_angle += 2 * np.pi * sign
    
    # in range, not end
    if (sign * now_z_angle) >= (sign * init_z_angle) and \
            (sign * now_z_angle) < (sign * end_z_angle):
        return False
    
    # out of range, end
    return True