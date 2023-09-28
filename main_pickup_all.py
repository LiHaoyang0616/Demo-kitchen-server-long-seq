from PIL import Image
import numpy as np
import math
from envs import TidyUpDish
import sapien.core as sapien
from utils.recording_gif import record_gif

SUPPORTED_ARM_TYPE = ['xarm6', 'xarm7']
SUPPORTED_HAND_TYPE = ['allegro','ability']
SUPPORTED_CONTROL_MODE = ['pd_joint_pos', 'pd_ee_pose', 'pd_ee_delta_pose']
TIME_INTERVAL = [100,300,500]

def normalize_hand_joints(env, hand_joints):
    qlimit_scope = env._qlimit_scope[env.arm_dof[0]:]
    print(qlimit_scope)
    normalized_joints = np.zeros_like(hand_joints)
    for i, (q_min, q_max) in enumerate(qlimit_scope):
        normalized_joints[i] = (2 * (hand_joints[i] - q_min) / (q_max - q_min)) - 1
    return normalized_joints

def main():
    arm_name = SUPPORTED_ARM_TYPE[1]
    control_mode = SUPPORTED_CONTROL_MODE[1]
    time_interval = TIME_INTERVAL[1]
    env = TidyUpDish(arm_name=arm_name,control_mode=control_mode, time_interval = time_interval)
    env.reset()
    # env.viewer.toggle_pause(paused=True) # True
    flag = True
    robot_left = env.robot[0]

    # init_mobile_base_action
    mobile_joints = [0.0,0.0,0.0]
    # init_arm_action
    arm_init_pos,arm_init_quat = [-0.201,0.350,0.292],[-0.707,0.0, 0.707, 0.0]
    arm_init_pose = np.concatenate([arm_init_pos,arm_init_quat])
    # init_hand_action
    hand_init_joints = [0.0,0.0]

    # # init_action
    action = np.zeros([(env.controller[0].action_dim)])
    action[0:env.mobile_dof[0]], action[env.mobile_dof[0]:env.arm_dof[0]+env.mobile_dof[0]], action[env.arm_dof[0]+env.mobile_dof[0]:] = mobile_joints, arm_init_pose, hand_init_joints
       
    start_step,end_step_1,end_step_2 = 5 + 5 * time_interval, 5 + 7 * time_interval, 5 + 7 * time_interval + 6450

    # while not env.viewer.closed:
    for step in range(start_step, end_step_2 + 1):
        if flag:
            if start_step <= step < end_step_1: 
                pickup_info = env.traj.pickup_teapot(step = step, robot = robot_left)
                env.switch_control_mode(robot = robot_left, arm_name = arm_name, control_mode=SUPPORTED_CONTROL_MODE[pickup_info["control_mode_index"]]) 
                action[0:env.mobile_dof[0]], action[env.mobile_dof[0]:env.mobile_dof[0]+env.arm_dof[0]], action[env.mobile_dof[0]+env.arm_dof[0]:] = pickup_info["mobile_joints"], pickup_info["arm_pose"], pickup_info["hand_joints"] 
                print(step)
            # save image:
                # env.camera.take_picture()
                # rgb = env.camera.get_color_rgba()
                # rgb_file = Image.fromarray((rgb * 255).astype(np.uint8))
                # name = 'images/pickupteapot_new/'+ str(step) + '.png'
                # rgb_file.save(name)

            elif end_step_1 <= step < end_step_2:
                env.robot[0].set_root_pose(sapien.Pose(p=[-1.131, 1.727, 0],q=[-0.696, 0.0, 0.0, 0.718]))
                t = step - end_step_1 + 5
                pickup_info = env.traj.open_fridge(step = t, robot = robot_left, banana=env.banana)
                env.switch_control_mode(robot = robot_left, arm_name = arm_name, control_mode=SUPPORTED_CONTROL_MODE[pickup_info["control_mode_index"]]) 
                action[0:env.mobile_dof[0]], action[env.mobile_dof[0]:env.mobile_dof[0]+env.arm_dof[0]], action[env.mobile_dof[0]+env.arm_dof[0]:] = pickup_info["mobile_joints"], pickup_info["arm_pose"], pickup_info["hand_joints"] 
                print(step)

            elif step == end_step_2:
                print("it is end !!!")
                record_gif()  # record gif
                flag = False

        env.step(action)
        # print("env.controller[1].action_dim",env.controller[1].action_dim)
        # print("step:", step)
        print("-------Action---------\n", action)
        # print("-------pose-----------\n", env.get_ee_pose())
        # print("-------Joints---------\n", np.array(env.robot_right.get_qpos()[0: env.arm_dof[0]]))

if __name__ == '__main__':
    main()