import sapien.core as sapien

from agents.controllers import DictController
from agents.configs.allegro_hand import XArmAllegroDefaultConfig

def set_up_controller(arm_name='xarm7', control_mode=None, robot=sapien.Articulation, **kwargs):

    config = XArmAllegroDefaultConfig(arm_name)
    
    controller = DictController(config=config, control_mode=control_mode, robot=robot, **kwargs)
    return controller