import gym
import numpy as np

import omnigibson as og
from omnigibson.envs.env_base import Environment


class EnvironmentAdvanced(Environment):
    
    def step(
        self,
        action=None,
        tgt_joint_pos=None,
        tgt_joint_idx=None,
    ):
        """
        Apply action or (tgt_joint_pos, tgt_joint_idx) to the robot

        Args:
            action (gym.spaces.Dict or dict or np.array): robot actions. If a dict is specified, each entry should
                map robot name to corresponding action. If a np.array, it should be the flattened, concatenated set
                of actions
            tgt_joint_pos (np.array): desired joint position of the robot
            tgt_joint_idx (np.array): selected joint index
            
            ps. action or (tgt_joint_pos, tgt_joint_idx) can be valid only one of them.

        # Returns:
        #     4-tuple:
        #         - dict: state, i.e. next observation
        #         - float: reward, i.e. reward at this current timestep
        #         - bool: done, i.e. whether this episode is terminated
        #         - dict: info, i.e. dictionary with any useful information
        """
        # action or (tgt_joint_pos, tgt_joint_idx) can be valid only one of them.
        assert not ((tgt_joint_pos is None) ^ (tgt_joint_idx is None))
        assert (tgt_joint_pos is None) ^ (action is None)
        
        # If the action is not a dictionary, convert into a dictionary
        if not isinstance(action, dict) and not isinstance(action, gym.spaces.Dict):
            action_dict = dict()
            idx = 0
            for robot in self.robots:
                action_dim = robot.action_dim
                action_dict[robot.name] = action[idx: idx + action_dim]
                idx += action_dim
        else:
            # Our inputted action is the action dictionary
            action_dict = action

        if action is not None:
            
            pass
        elif tgt_joint_pos is not None:
            # Only support one robot currently
            self.robots[0].set_joint_positions(
                tgt_joint_pos, 
                indices=tgt_joint_idx,
                drive=True,
            )

        # Run simulation step
        og.sim.step()