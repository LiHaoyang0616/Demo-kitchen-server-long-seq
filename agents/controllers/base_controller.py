import numpy as np
import sapien.core as sapien
from typing import Dict, List
from dataclasses import dataclass

from .utils import flatten_action_spaces, get_active_joint_indices, get_active_joints


class BaseController:

    joints: List[sapien.Joint]  # active joints controlled
    joint_indices: List[int]  # indices of active joints controlled

    def __init__(self, config, robot, start, end, **kwargs):
        self.config = config
        self.robot = robot
        self.start_index, self.end_index = start, end   # Start and end index of qpos of current robot.
        self.joints = robot.get_active_joints()

    @property
    def qpos(self):
        """Get current joint positions."""
        return self.robot.get_qpos()[self.start_index:self.end_index]

    @property
    def qvel(self):
        """Get current joint velocities."""
        return self.robot.get_qvel()[self.start_index:self.end_index]
    
    @property
    def qlimits(self):
        """Get qlimits."""
        return self.robot.get_qlimits()[self.start_index:self.end_index]

    # -------------------------------------------------------------------------- #
    # Interfaces (implemented in subclasses)
    # -------------------------------------------------------------------------- #
    def reset(self):
        """Reset the controller.
        """
        raise NotImplementedError

    def set_target(self, action: np.ndarray):
        """Set the action to execute.
        The action can be low-level control signals or high-level abstract commands.
        """
        raise NotImplementedError
    
    def get_target_qpos(self):
        """Get target qpos.
        """
        raise NotImplementedError
    
    def get_target_ee_pose(self):
        """Get target end effector's pose
        """
        raise NotImplementedError
    
    def get_ee_pose(self):
        """Get end effector's pose
        """
        raise NotImplementedError

    def _clip_action(self, action, low, high):
        """Clip action to [low, high]."""
        action = np.clip(action, low, high)
        return action

    def _clip_and_scale_action(self, action, low, high):
        """Clip action to [-1, 1] and scale according to a range [low, high]."""
        low, high = np.asarray(low), np.asarray(high)
        action = np.clip(action, -1, 1)
        return 0.5 * (high + low) + 0.5 * (high - low) * action
    
    def set_drive_property(self):
        """Set the joint drive property according to the config."""
        raise NotImplementedError

    def _preprocess_action(self, action: np.ndarray):
        # TODO(jigu): support discrete action
        if self.config['normalize_action']:
            action = self._clip_and_scale_action(action)
        return action

    def _initialize_action_space(self):
        # self.action_space = spaces.Box(...)
        raise NotImplementedError
    
@dataclass
class ControllerConfig:
    joint_names: List[str]
    # NOTE(jigu): It is a class variable in this base class,
    # but you can inherit it and overwrite with an instance variable.
    controller_cls = BaseController

class DictController:
    def __init__(self, config, control_mode, robot:sapien.Articulation, **kwargs):
        """
            Integrate arm controller and hand controller.
        """
        config = config[control_mode]
        self.robot = robot
        self.joints = robot.get_active_joints()
        
        arm_dof, hand_dof, mobile_dof = config['arm']['arm_dof'], config['hand']['hand_dof'], config['mobile']['mobile_dof']
        self.robot_dof = arm_dof + hand_dof + mobile_dof

        arm_action_dim, hand_action_dim, mobile_action_dim = config['arm']['action_dim'], config['hand']['action_dim'], config['mobile']['action_dim']
        self.action_dim = arm_action_dim + hand_action_dim + mobile_action_dim
        self.action_mapping = dict(mobile=[0, mobile_action_dim], arm=[mobile_action_dim, arm_action_dim + mobile_action_dim], hand = [arm_action_dim + mobile_action_dim, self.action_dim])

        mobile_controller_cls = config['mobile']['controller_cls']
        mobile_controller = mobile_controller_cls(config=config['mobile'], robot=robot,
                                            start=0, end=mobile_dof, **kwargs)

        arm_controller_cls = config['arm']['controller_cls']
        arm_controller = arm_controller_cls(config=config['arm'], robot=robot, 
                                            start=mobile_dof, end=mobile_dof+arm_dof, **kwargs)
        
        hand_controller_cls = config['hand']['controller_cls']
        hand_controller = hand_controller_cls(config=config['hand'], robot=robot, 
                                            start=mobile_dof+arm_dof, end=self.robot_dof, **kwargs)

        self.dict_controller = dict(mobile=mobile_controller, arm=arm_controller, hand=hand_controller)

    
    def set_action(self, action):
        mobile_action = self.dict_controller['mobile'].set_target(action[self.action_mapping['mobile'][0]:self.action_mapping['mobile'][1]])
        arm_action = self.dict_controller['arm'].set_target(action[self.action_mapping['arm'][0]:self.action_mapping['arm'][1]])
        hand_action = self.dict_controller['hand'].set_target(action[self.action_mapping['hand'][0]:self.action_mapping['hand'][1]])
        action = np.concatenate([mobile_action, arm_action, hand_action])

        # NOTE(haoyang)：
        '''
        Action space:
            0-2: mobile base
            3-9: arm
            10-11: hand
        '''

        for i, joint in enumerate(self.joints):
            joint.set_drive_target(action[i])

    def reset(self):
        self.dict_controller['arm'].reset()
        self.dict_controller['hand'].reset()
        # self.dict_controller['mobile'].reset()

    