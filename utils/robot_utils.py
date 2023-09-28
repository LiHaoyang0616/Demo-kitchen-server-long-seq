import numpy as np
import sapien.core as sapien

from pathlib import Path
from typing import Dict, NamedTuple, List
from copy import deepcopy

class RobotInfo(NamedTuple):
    path: str
    arm_dof: int
    hand_dof: int
    mobile_dof: int
    palm_name: str
    arm_init_qpos: List[float]
    # root_offset: List[float] = [0.0, 0.0, 0.0]

def generate_robot_info():
    xarm6_path = Path("./assets/robot/xarm6_description/")
    xarm6_allegro_hand_left = RobotInfo(path=str(xarm6_path / "xarm6_allegro_long_finger_tip_left.urdf"), arm_dof=6, hand_dof=16, mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, -np.pi / 2, np.pi])
    xarm6_allegro_hand_right = RobotInfo(path=str(xarm6_path / "xarm6_allegro_long_finger_tip_right.urdf"), arm_dof=6, hand_dof=16, mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, -np.pi / 2, 0])
    xarm6_ability_hand_left = RobotInfo(path=str(xarm6_path / "xarm6_ability_left.urdf"), arm_dof=6, hand_dof=10,mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, -np.pi / 2, np.pi])
    xarm6_ability_hand_right = RobotInfo(path=str(xarm6_path / "xarm6_ability_right.urdf"), arm_dof=6, hand_dof=10, mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, -np.pi / 2, 0])
    

    xarm7_path = Path("./assets/robot/xarm7_description/")
    xarm7_allegro_hand_left = RobotInfo(path=str(xarm7_path / "xarm7_allegro_long_finger_tip_left.urdf"), arm_dof=7, hand_dof=16, mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, 0, -np.pi / 2, np.pi])
    xarm7_allegro_hand_right = RobotInfo(path=str(xarm7_path / "xarm7_allegro_long_finger_tip_right.urdf"), arm_dof=7, hand_dof=16,mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, 0, -np.pi / 2, 0])
    xarm7_ability_hand_left = RobotInfo(path=str(xarm7_path / "xarm7_ability_left.urdf"), arm_dof=7, hand_dof=10, mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, 0, -np.pi / 2, np.pi])
    xarm7_ability_hand_right = RobotInfo(path=str(xarm7_path / "xarm7_ability_right.urdf"), arm_dof=7, hand_dof=10,mobile_dof = 0,
                                        palm_name="palm_center", arm_init_qpos=[0, 0, 0, 0, 0, -np.pi / 2, 0])
    
    xarm7_reduced_gripper_mobile = RobotInfo(path=str(xarm7_path / "xarm_grasper_urdf/xarm7_reduced_gripper_mobile.urdf"), arm_dof=7, hand_dof=2, mobile_dof = 3,
                                        palm_name="palm_center", arm_init_qpos=[-2.5965488, -0.20909688, 0.0058131623, 0.035645034, 0.038908407, -1.3677506, -1.615564])
    # pick_teapot: [0, 0, 0, 0, 0, -np.pi / 2, 0]
    # open_fridge: [-2.5965488, -0.20909688, 0.0058131623, 0.035645034, 0.038908407, -1.3677506, -1.615564]
    

    info_dict = dict(xarm6_allegro_hand_left=xarm6_allegro_hand_left, 
                     xarm6_allegro_hand_right=xarm6_allegro_hand_right,
                     xarm6_ability_hand_left=xarm6_ability_hand_left,
                     xarm6_ability_hand_right=xarm6_ability_hand_right,
                     xarm7_allegro_hand_left=xarm7_allegro_hand_left,
                     xarm7_allegro_hand_right=xarm7_allegro_hand_right,
                     xarm7_ability_hand_left=xarm7_ability_hand_left,
                     xarm7_ability_hand_right=xarm7_ability_hand_right,
                     xarm7_reduced_gripper_mobile=xarm7_reduced_gripper_mobile,)

    return info_dict
def parse_urdf_config(config_dict: dict, scene: sapien.Scene) -> Dict:
    """Parse config from dict for SAPIEN URDF loader.

    Args:
        config_dict (dict): a dict containing link physical properties.
        scene (sapien.Scene): simualtion scene

    Returns:
        Dict: urdf config passed to `sapien.URDFLoader.load`.
    """
    urdf_config = deepcopy(config_dict)

    # Create the global physical material for all links
    mtl_cfg = urdf_config.pop("material", None)
    if mtl_cfg is not None:
        urdf_config["material"] = scene.create_physical_material(**mtl_cfg)

    # Create link-specific physical materials
    materials = {}
    for k, v in urdf_config.pop("_materials", {}).items():
        materials[k] = scene.create_physical_material(**v)

    # Specify properties for links
    for link_config in urdf_config.get("link", {}).values():
        # Substitute with actual material
        link_config["material"] = materials[link_config["material"]]

    return urdf_config

def load_robot(scene: sapien.Scene, robot_name: str, disable_self_collision: bool = False) -> sapien.Articulation:
    """
        Load robot, set up collsion, drive(control) and visual property.
    """
    # Load robot
    loader = scene.create_urdf_loader()
    info = generate_robot_info()[robot_name]
    filename = info.path

    urdf_config = dict(
        _materials=dict(
            gripper=dict(static_friction=10.0, dynamic_friction=10.0, restitution=0.0)
        ),
        link=dict(
            left_finger=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            right_finger=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
        ),
    )
    
    urdf_config = parse_urdf_config(urdf_config, scene)

    robot = loader.load(filename, config=urdf_config)

    # robot_builder = loader.load_file_as_articulation_builder(filename)
    # # Set up collision property
    # if 'allegro' in robot_name:
    #     if disable_self_collision:
    #         for link_builder in robot_builder.get_link_builders():
    #             link_builder.set_collision_groups(1, 1, 17, 0)
    #     else:
    #         for link_builder in robot_builder.get_link_builders():
    #             # NOTE(chichu): These links are at the junction of palm and fingers
    #             if link_builder.get_name() in ["link_9.0", "link_5.0", "link_1.0", "link_13.0", "base_link"]:
    #                 link_builder.set_collision_groups(1, 1, 17, 0)
    # elif 'ability' in robot_name:
    #     pass
    # # else:
    # #     raise NotImplementedError
    # robot = robot_builder.build(fix_root_link=True)

    robot.set_name(robot_name)
    # Set up drive(control) property
    arm_control_params = np.array([1e5, 4e4, 5e4])  # This PD is far larger than real to improve stability # [2e5, 4e4, 5e2]
    hand_control_params = np.array([1e3, 1e2, 1e4])
    mobile_control_params = np.array([1e3, 4e2, 5e4])

    arm_joint_names = [f"joint{i}" for i in range(1, 8)]    # NOTE(chichu):This setting is compataible with both xarm6 and xarm7.
    mobile_joint_names = ["root_x_axis_joint", "root_y_axis_joint", "root_z_rotation_joint"]


    for joint in robot.get_active_joints():
        name = joint.get_name()
        if name in arm_joint_names:
            joint.set_drive_property(*(1 * arm_control_params), mode="force")
        elif name in mobile_joint_names:
            joint.set_drive_property(*(1 * mobile_control_params), mode="force")
        else:
            joint.set_drive_property(*(1 * hand_control_params), mode="force")
            
    # Set up visual material
    mat_physi = scene.engine.create_physical_material(1.5, 1, 0.01)
    for link in robot.get_links():
        for geom in link.get_collision_shapes():
            geom.min_patch_radius = 0.02
            geom.patch_radius = 0.04
            geom.set_physical_material(mat_physi)

    return robot