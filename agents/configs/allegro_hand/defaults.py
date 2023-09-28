import numpy as np
from agents.controllers import PDJointPosController, PDEEPoseController, PDBaseVelController

def XArmAllegroDefaultConfig(arm_name='xarm7'):
        use_target = True
        arm_dof = int(arm_name[-1])
        hand_dof = 2
        # Arm
        arm_pd_joint_pos = dict(lower=None, upper=None, 
                                normalize_action=True, 
                                use_delta=False,
                                use_target=use_target,
                                action_dim=arm_dof,
                                arm_dof=arm_dof,
                                controller_cls=PDJointPosController)
        arm_pd_ee_pose = dict(lower=None, upper=None, 
                                normalize_action=False,
                                use_delta=False, 
                                use_target=None,
                                action_dim=7,
                                arm_dof=arm_dof,
                                controller_cls=PDEEPoseController)
        arm_pd_ee_delta_pose = dict(lower=-1, upper=1, rot_bound=np.pi/2, 
                                    normalize_action=True,
                                    use_delta=True,
                                    use_target=use_target, 
                                    action_dim=6,
                                    arm_dof=arm_dof,
                                    controller_cls=PDEEPoseController)
        # Hand
        hand_pd_joint_pos = dict(lower=None, upper=None, 
                                normalize_action=True, 
                                use_delta=False,
                                use_target=use_target,
                                action_dim=2,
                                hand_dof=2,
                                controller_cls=PDJointPosController)
        # Movable Base
        mobile_base_pd_joint_pos = dict(lower=None, upper=None,
                                        normalize_action=True, 
                                        use_delta=False,
                                        use_target=use_target,
                                        mobile_dof=3,
                                        action_dim=3,
                                        controller_cls=PDJointPosController)
        
        controller_configs = dict(
            pd_joint_pos=dict(arm=arm_pd_joint_pos, hand=hand_pd_joint_pos, mobile=mobile_base_pd_joint_pos),
            pd_ee_pose=dict(arm=arm_pd_ee_pose, hand=hand_pd_joint_pos, mobile=mobile_base_pd_joint_pos),
            pd_ee_delta_pose=dict(arm=arm_pd_ee_delta_pose, hand=hand_pd_joint_pos, mobile=mobile_base_pd_joint_pos),
        )

        return controller_configs