from dataclasses import dataclass
from typing import Sequence, Union

import numpy as np
from gym import spaces

from .base_controller import BaseController, ControllerConfig


class PDJointVelController(BaseController):
    config: "PDJointVelControllerConfig"

    def _initialize_action_space(self):
        n = len(self.joints)
        low = np.float32(np.broadcast_to(self.config.lower, n))
        high = np.float32(np.broadcast_to(self.config.upper, n))
        self.action_space = spaces.Box(low, high, dtype=np.float32)

    def set_drive_property(self):
        n = len(self.joints)
        damping = np.broadcast_to(self.config.damping, n)
        force_limit = np.broadcast_to(self.config.force_limit, n)
        friction = np.broadcast_to(self.config.friction, n)

        for i, joint in enumerate(self.joints):
            joint.set_drive_property(0, damping[i], force_limit=force_limit[i])
            joint.set_friction(friction[i])

    def set_target(self, action: np.ndarray):
        action = self._preprocess_action(action)
        self._target_qpos = action
        return action


@dataclass
class PDJointVelControllerConfig(ControllerConfig):
    lower: Union[float, Sequence[float]]
    upper: Union[float, Sequence[float]]
    damping: Union[float, Sequence[float]]
    force_limit: Union[float, Sequence[float]] = 1e10
    friction: Union[float, Sequence[float]] = 0.0
    normalize_action: bool = True
    controller_cls = PDJointVelController
