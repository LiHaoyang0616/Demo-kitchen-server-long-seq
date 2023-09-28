import numpy as np
from sapien.core import Pose
from numpy import pi
from scipy.spatial.transform import Rotation as R
import sapien.core as sapien

class Trajectory():
  def __init__(self,arm_name='xarm7', time_interval = 10, qlimit_scope = None):
    self.arm_dof = int(arm_name[-1])
    self.hand_dof = 2
    self.mobile_dof = 3

    self.arm_name = arm_name
    self._hand_init_joints = None

    self._qlimit_scope = qlimit_scope
    self.time_interval = time_interval
    self.mobile_dist_list = [] 

  def pickup_teapot(self, robot:sapien.ArticulationBase, step):
    if (step-5) % self.time_interval == 0 :
      self._cur_base_qpos = self.normalize_mobile_joints(np.array(robot.get_qpos()[0: self.mobile_dof]))
      self._cur_arm_qpos = self.normalize_arm_joints(np.array(robot.get_qpos()[self.mobile_dof: self.mobile_dof+self.arm_dof]))
      self._cur_hand_qpos = self.normalize_hand_joints(np.array(robot.get_qpos()[self.mobile_dof+self.arm_dof:]))
      self._cur_ee_pose = [link for link in robot.get_links() if link.get_name() == 'xarm_gripper_base_link'][0].get_pose()

    self._hand_init_joints = self.normalize_hand_joints([0.023,0.023]) 

    # get close to the teapot
    if 5 <= step <= 5 + 1.0 * self.time_interval:
      control_mode_index = 0
      step = (step-5) / (1 * self.time_interval)

      arm_joints = self._cur_arm_qpos

      mobile_joints = self.normalize_mobile_joints(np.array([0.0, 1.324, 0.0]))

      hand_joints = self._hand_init_joints

    # init prepare pose for teapot
    elif 5 + 1 * self.time_interval < step <= 5 + 1.5 * self.time_interval:
      step = (step-(5 + 1 * self.time_interval)) / (0.5 * self.time_interval)
      control_mode_index = 0

      arm_next_joints = np.array([0.0, 0.477, 0.0, 1.149, 0.0, -0.85, -1.607])
      arm_joints = self.normalize_arm_joints(arm_next_joints)

      mobile_joints = self.normalize_mobile_joints(np.array([0.077, 1.324, 0.0]))

      hand_joints = self._hand_init_joints

    # init pose for pickup teapot
    elif 5 + 1.5 *  self.time_interval < step <= 5 + 2.0 * self.time_interval:
      print("=== Phase: 3 ===")
      step = (step-(5 + 1.5 *  self.time_interval)) / (0.5 * self.time_interval)
      control_mode_index = 1

      arm_trans = self.interpolate_pos(step, np.array([-0.527,-0.971,0.355]),np.array([-0.600,-0.971,0.318]), 0, 1)
      arm_rot = np.array([-0.479,-0.521,0.503,0.496])

      mobile_joints = self.normalize_mobile_joints(np.array([0.077, 1.324, 0.0]))

      hand_joints = self._hand_init_joints

    # pickup teapot
    elif 5 + 2.0 *  self.time_interval < step <= 5 + 3.0 * self.time_interval:
      print("=== Phase: 3 ===")
      step = (step-(5 + 2.0 *  self.time_interval)) / (1 * self.time_interval)
      control_mode_index = 1

      arm_trans = np.array([-0.608,-0.971,0.318])
      arm_rot = np.array([-0.479,-0.521,0.503,0.496])

      mobile_joints = self.normalize_mobile_joints(np.array([0.077, 1.324, 0.0]))

      hand_joints = self.interpolate_pos(step, self._hand_init_joints, self.normalize_hand_joints(np.array([0.008,0.008])), 0, 1)

    # lift teapot
    elif 5 + 3.0 * self.time_interval < step <= 5 + 4.0 * self.time_interval:
      print("=== Phase: 4 ===")
      step = (step-(5 + 3.0 * self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 1

      arm_trans = self.interpolate_pos(step,np.array([-0.608,-0.971,0.318]),np.array([-0.608,-0.971,0.435]),0,1)
      arm_rot = np.array([-0.479,-0.521,0.503,0.496])

      mobile_joints = self.normalize_mobile_joints(np.array([0.077, 1.324, 0.0]))

      hand_joints = self.normalize_hand_joints(np.array([0.008,0.008]))

    # place teapot
    elif 5 + 4.0 * self.time_interval < step <= 5 + 5.0 * self.time_interval:
      print("=== Phase: 5 ===")
      step = (step-(5 + 4.0 * self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 0

      arm_next_joints = np.array([-2.63, 0.042, 0.0, 0.862, -8.868643e-07, -0.794, -1.607])
      arm_joints = self.interpolate_pos(step,self.normalize_arm_joints(np.array([-0.06656983, 0.5902049, 0.12393617, 1.5908283, 0.08051561, -0.51876163, -1.7426602])),self.normalize_arm_joints(arm_next_joints),0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.0, 1.324, 0.0]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([-0.721, -0.254, -0.495]))
      mobile_joints = self.interpolate_pos(step,mobile_prev_joints,mobile_next_joints,0,1)

      hand_joints = self.normalize_hand_joints(np.array([0.008,0.008]))

    # put down teapot
    elif 5 + 5.0 * self.time_interval < step <= 5 + 5.5 * self.time_interval:
      print("=== Phase: 6 ===")
      step = (step-(5 + 5.0 * self.time_interval)) / (0.5 * self.time_interval)
      control_mode_index = 0

      arm_next_joints = self.normalize_arm_joints(np.array([-2.6247978, 0.523, 0.00030472426, 1.037, 0.00012675372, -1.105, -1.6072831]))
      arm_joints = arm_next_joints

      mobile_joints = self.normalize_mobile_joints(np.array([-0.721, -0.254, -0.495]))

      hand_joints = self.normalize_hand_joints(np.array([0.008,0.008]))

    # leave the teapot
    elif 5 + 5.5 * self.time_interval < step <= 5 + 6.0 * self.time_interval:
      print("=== Phase: 7 ===")
      step = (step-(5 + 5.5 * self.time_interval)) / (0.5 * self.time_interval)
      control_mode_index = 0

      arm_next_joints = self.normalize_arm_joints(np.array([-2.6247978, 0.523, 0.00030472426, 1.037, 0.00012675372, -1.105, -1.6072831]))
      arm_joints = arm_next_joints

      mobile_prev_joints = self.normalize_mobile_joints(np.array([-0.721, -0.254, -0.495]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([-0.487, 0.0, -0.043]))
      mobile_joints = self.interpolate_pos(step,mobile_prev_joints,mobile_next_joints,0,1)

      hand_joints = np.array([0.044,0.044])

    # close to the refrigerator
    elif 5 + 6.0 * self.time_interval < step <= 5 + 7.0 * self.time_interval:
      print("=== Phase: 7 ===")
      step = (step-(5 + 6.0 * self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 0

      arm_prev_joints = self._cur_arm_qpos
      arm_next_joints = self.normalize_arm_joints(np.array([3.39, 0.096, -1.274, 0.587, 0.281, -0.20503911, -0.015724707]))
      arm_joints = self.interpolate_pos(step,arm_prev_joints,arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([-0.487, 0.0, -0.043]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([1.166, -1.381, 1.545]))
      mobile_joints = self.interpolate_pos(step,mobile_prev_joints,mobile_next_joints,0,1)

      hand_joints = np.array([0.044,0.044])

    # close to the refrigerator
    elif 5 + 7.0 * self.time_interval < step <= 5 + 8.0 * self.time_interval:
      print("=== Phase: 8 ===")
      step = (step-(5 + 7.0 * self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 0

      arm_prev_joints = self._cur_arm_qpos
      arm_next_joints = self.normalize_arm_joints(np.array([3.39, 0.096, -1.274, 0.587, 0.281, -0.20503911, -0.015724707]))
      arm_joints = arm_next_joints # self.interpolate_pos(step,arm_prev_joints,arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([-0.487, 0.0, -0.043]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([1.166, -1.381, 1.545]))
      mobile_joints = mobile_next_joints # self.interpolate_pos(step,mobile_prev_joints,mobile_next_joints,0,1)

      hand_joints = np.array([0.044,0.044])

    else:
      print("Wrong step")
      raise NotImplementedError
    
    if control_mode_index == 1:
      arm_pose = np.zeros(7)
      arm_pose[0:3], arm_pose[3:7] = arm_trans, arm_rot
    elif control_mode_index == 2:
      arm_pose = np.zeros(6)
      arm_pose[0:3], arm_pose[3:6] = arm_trans, arm_rot
    elif control_mode_index == 0:
      arm_pose = np.zeros(self.arm_dof)
      arm_pose[0:self.arm_dof] = arm_joints
      
    else:
      raise NotImplementedError
    
    key_point_info = dict(
        arm_pose=arm_pose,
        hand_joints= hand_joints,
        mobile_joints = mobile_joints,
        control_mode_index= control_mode_index
      )
    
    return key_point_info

  def open_fridge(self, robot:sapien.ArticulationBase, banana:sapien.Actor,step):
    phase_end_step = [117,417,808,1108,1408,1588,1731,2031,2299,2631,3082,3472,3972,4422,4708,5108,5337,5537,5837,5990,6150,6450]
    phase_lenth = [112, 300, 391, 300, 300, 180, 143, 300, 268, 332, 451, 390, 500, 450, 286, 400, 229, 200, 300, 153, 160, 300]

    if step in phase_end_step or step == 5:
      self._cur_base_qpos = self.normalize_mobile_joints(np.array(robot.get_qpos()[0: self.mobile_dof]))
      self._cur_arm_qpos = self.normalize_arm_joints(np.array(robot.get_qpos()[self.mobile_dof: self.mobile_dof+self.arm_dof]))
      self._cur_hand_qpos = self.normalize_hand_joints(np.array(robot.get_qpos()[self.mobile_dof+self.arm_dof:]))
      self._cur_ee_pose = [link for link in robot.get_links() if link.get_name() == 'xarm_gripper_base_link'][0].get_pose()
    
    self._hand_init_joints = self.normalize_hand_joints(np.array([0.044,0.044]))
    
    # get close to the fridge
    if 5 <= step <= phase_end_step[0]:
      control_mode_index = 0
      step = (step-5) / (phase_lenth[0])
      
      arm_prev_joints = self._cur_arm_qpos
      arm_next_joints = self.normalize_arm_joints(np.array([3.39, 0.096, -1.274, 0.587, 0.281, -0.20503911, -0.015724707]))
      arm_joints = arm_next_joints # self.interpolate_pos(step,arm_prev_joints,arm_next_joints, 0, 1)

      mobile_prev_joints = self._cur_base_qpos
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.007, -0.048, 0.028055714]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) # self.normalize_mobile_joints(np.array([0.241, 0.351, -0.146])) # self.normalize_mobile_joints(self.interpolate_pos(step, np.array([0.0, 0.0, 0.0]), np.array([0.294, -0.037, -0.393]), 0, 1)) # np.array([0.0, -0.184, -0.358]) # self.interpolate_pos(step, self._cur_base_qpos, np.array([0.0, -0.184, -0.358]), 0, 1)

      list = self.euclidean_distance(mobile_prev_joints,mobile_next_joints)

      hand_joints = self._hand_init_joints

    # catch the door
    elif phase_end_step[0] < step <= phase_end_step[1]:
      step = (step-phase_end_step[0]) / phase_lenth[1]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([3.39, 0.096, -1.274, 0.587, 0.281, -0.20503911, -0.015724707]))
      arm_next_joints = self.normalize_arm_joints(np.array([3.3900173, 0.09599977, -1.274, 1.011, 0.28099725, -0.20503762, -0.01572459]))
      arm_joints = arm_next_joints

      mobile_joints = self.normalize_mobile_joints(np.array([0.007, -0.048, 0.028055714]))

      hand_joints = self.interpolate_pos(step, self._cur_hand_qpos, self.normalize_hand_joints(np.array([0.017,0.007])), 0, 1) 

    # open the fridge
    elif phase_end_step[1] < step <= phase_end_step[2]:
      step = (step - phase_end_step[1]) / phase_lenth[2]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([3.3900173, 0.09599977, -1.274, 1.011, 0.28099725, -0.20503762, -0.01572459]))
      arm_next_joints = self.normalize_arm_joints(np.array([2.7207, 0.09593805, -1.2741592, 0.898064, 0.108, -0.20498519, -0.015708191]))
      arm_joints = arm_next_joints # self.interpolate_pos(step,arm_prev_joints,arm_next_joints, 0, 1)

      mobile_prev_joints = self._cur_base_qpos
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.07403012, 0.403, -1.009]))
      mobile_joints = mobile_next_joints # self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1)
      
      hand_joints =self.normalize_hand_joints(np.array([0.044, 0.005]))

    elif phase_end_step[2] < step <= phase_end_step[3]:
      step = (step - phase_end_step[2]) / phase_lenth[3]

      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([2.7207, 0.09593805, -1.2741592, 0.898064, 0.108, -0.20498519, -0.015708191]))
      arm_next_joints = self.normalize_arm_joints(np.array([2.958, 0.482, -0.41, 1.393, 0.108000085, 0.915, 1.447]))
      arm_joints =  self.interpolate_pos(step,arm_prev_joints,arm_next_joints,0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.07403012, 0.403, -1.009]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.074030876, 0.584, 0.308]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1)

      hand_joints = self.normalize_hand_joints(np.array([0.044, 0.044]))

    # down to catch banana
    elif phase_end_step[3] < step <= phase_end_step[4]:
      step = (step - phase_end_step[3]) / phase_lenth[4]
      control_mode_index = 1

      arm_trans = self.interpolate_pos(step, self._cur_ee_pose.p, np.array([-0.398,2.191,0.189]), 0, 1)
      arm_rot = np.array([0.008,-0.993,0.106,-0.052])

      mobile_joints = self.normalize_mobile_joints(np.array([0.074030876, 0.584, 0.308]))

      hand_joints = self.interpolate_pos(step, self._cur_hand_qpos, self.normalize_hand_joints(np.array([0.008,0.008])), 0, 1) 

    # leave banana
    elif phase_end_step[4] < step <= phase_end_step[5]:
      step = (step - phase_end_step[4]) / phase_lenth[5]

      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([2.9240868, 0.6265625, -0.37228203, 1.3492223, 0.1716578, 0.7281945, 1.4192367]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.453, 0.767, -0.324, 2.101, 0.238, 1.314, 1.447]))
      arm_joints = self.interpolate_pos(step,arm_prev_joints,arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.074030876, 0.584, 0.308]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.241, 0.162, -0.146]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) # self.normalize_mobile_joints(np.array([0.241, 0.351, -0.146])) # self.normalize_mobile_joints(self.interpolate_pos(step, np.array([0.0, 0.0, 0.0]), np.array([0.294, -0.037, -0.393]), 0, 1)) # np.array([0.0, -0.184, -0.358]) # self.interpolate_pos(step, self._cur_base_qpos, np.array([0.0, -0.184, -0.358]), 0, 1)

      hand_joints =self.normalize_hand_joints(np.array([0.008, 0.008]))

   
    # place banana
    elif phase_end_step[5] < step <= phase_end_step[6]:
      step = (step - phase_end_step[5]) / phase_lenth[6]
      
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.453, 0.767, -0.324, 2.101, 0.238, 1.314, 1.447]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.45629707, 1.01, -0.3237881, 2.328, 0.2377447, 1.347, 1.835]))
      arm_joints =self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.241, 0.162, -0.146]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.227, 0.46, 0.221]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.044,0.044]))

    # catch the fridge door
    elif phase_end_step[6] < step <= phase_end_step[7]:
      step = (step - phase_end_step[6]) / phase_lenth[7]

      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.45629707, 1.01, -0.3237881, 2.328, 0.2377447, 1.347, 1.835]))
      arm_next_joints = self.normalize_arm_joints(np.array([1.188, 0.981, -0.713, 2.299, 2.008, 2.028, 1.365]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_joints = self.normalize_mobile_joints(np.array([0.227, 0.46, 0.221]))

      hand_joints = self.interpolate_pos(step, self.normalize_hand_joints(np.array([0.060,0.060])), self.normalize_hand_joints(np.array([0.005,0.005])), 0, 1) 

    # close door
    elif phase_end_step[7] < step <= phase_end_step[8]:
      step = (step - phase_end_step[7]) / phase_lenth[8]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([1.188, 0.981, -0.713, 2.299, 2.008, 2.028, 1.365]))
      arm_next_joints = self.normalize_arm_joints(np.array([1.1880105, 0.9949403, -0.3671315, 2.298953, 2.0080385, 1.43, 1.5759776]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.227, 0.46, 0.221]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.089, -0.254, 0.8909949]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.010,0.010])) # self.interpolate_pos(step, self.normalize_hand_joints(np.array([0.010,0.010])), self.normalize_hand_joints(np.array([0.040,0.040])), 0, 1) 

    # move to the place 
    elif phase_end_step[8] < step <= phase_end_step[9]:
      step = (step - phase_end_step[8]) / phase_lenth[9]

      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([1.1880105, 0.9949403, -0.3671315, 2.298953, 2.0080385, 1.43, 1.5759776]))
      arm_next_joints = self.normalize_arm_joints(np.array([1.965, 0.439, -0.36713198, 0.686, 0.281, 0.101, -0.54])) 
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

    
      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.089, -0.254, 0.8909949]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.007, -0.249, 0.016]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.040,0.040])) 

    # turn right 
    elif phase_end_step[9] < step <= phase_end_step[10]:
      step = (step - phase_end_step[9]) / phase_lenth[10]

      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([1.965, 0.439, -0.36713198, 0.686, 0.281, 0.101, -0.54]))
      arm_next_joints = self.normalize_arm_joints(np.array([1.9342264, -0.503, -0.36718896, 0.94, 0.34861392, 0.799, -0.151]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)
      
      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.007, -0.249, 0.016]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.447, 1.134, -1.085]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.040,0.040])) 

    # close to the banana 
    elif phase_end_step[10] < step <= phase_end_step[11]:
      step = (step - phase_end_step[10]) / phase_lenth[11]

      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([1.9342264, -0.503, -0.36718896, 0.94, 0.34861392, 0.799, -0.151]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.264, 0.933, -0.24883711, 2.318766, 0.22291082, 1.1027701, 0.12949224]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.447, 1.134, -1.085]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.842, 1.093, -1.571]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.040,0.040]))

    # close to pick banana
    elif phase_end_step[11] < step <= phase_end_step[12]:
      step = (step - phase_end_step[11]) / phase_lenth[12]
      
      control_mode_index = 1

      banana_pose = banana.get_pose()

      arm_trans = self.interpolate_pos(step,self._cur_ee_pose.p,np.array([banana_pose.p[0],banana_pose.p[1]+0.041,0.267]),0,1)
      arm_rot = self.slerp_quat(step,self._cur_ee_pose.q,np.array([-0.089,0.069,0.994,-0.008]),0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.842, 1.093, -1.571]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([0.842, 1.045, -1.571]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.040,0.040]))

    # lift banana
    elif phase_end_step[12] < step <= phase_end_step[13]:
      step = (step - phase_end_step[12]) / phase_lenth[13]

      control_mode_index = 0

      arm_prev_joints = self._cur_arm_qpos
      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_joints = self._cur_base_qpos

      hand_joints =self.normalize_hand_joints(np.array([0.008, 0.008]))

    # go to the target place
    elif phase_end_step[13] < step <= phase_end_step[14]:
      step = (step - phase_end_step[13]) / phase_lenth[14]

      control_mode_index = 0

      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints =  self.interpolate_pos(step,self._cur_arm_qpos,arm_next_joints,0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([0.842, 1.045, -1.571]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([3.247, 1.048, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.008, 0.008]))

    # put down banana
    elif phase_end_step[14] < step <= phase_end_step[15]:
      step = (step - phase_end_step[14]) / phase_lenth[15]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.40829623, 1.081, -0.40938967, 2.441, 0.324, 1.364, -0.065104134]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_joints = self.normalize_mobile_joints(np.array([3.247, 1.048, -1.538]))

      hand_joints = self.interpolate_pos(step, self.normalize_hand_joints(np.array([0.015, 0.015])), self.normalize_hand_joints(np.array([0.020,0.020])), 0, 1) 

    # go to the pan
    elif phase_end_step[15] < step <= phase_end_step[16]:
      step = (step - phase_end_step[15]) / phase_lenth[16]

      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.40829623, 1.081, -0.40938967, 2.441, 0.324, 1.364, -0.065104134]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 0.229, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints =  arm_next_joints # self.interpolate_pos(step,arm_prev_joints,arm_next_joints,0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([3.247, 1.048, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([1.323, 1.048, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.044, 0.044]))
   
    # close to the pan
    elif phase_end_step[16] < step <= phase_end_step[17]:
      step = (step- phase_end_step[16]) / phase_lenth[17]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.41, 0.229, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.2235833, 0.753, -0.29894835, 1.8524919, 0.14503075, 1.165, -0.064994514]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([1.34, 1.048, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([1.34, 1.22, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 
      
      hand_joints = self.interpolate_pos(step, self.normalize_hand_joints(np.array([0.044, 0.044])), self.normalize_hand_joints(np.array([0.025,0.025])), 0, 1) 
    
    # catch the pan
    elif phase_end_step[17] < step <= phase_end_step[18]:
      step = (step- phase_end_step[17]) / phase_lenth[18]
      control_mode_index = 0

      arm_joints = self.normalize_arm_joints(np.array([0.2235833, 0.753, -0.29894835, 1.8524919, 0.14503075, 1.165, -0.064994514]))

      mobile_joints = self.normalize_mobile_joints(np.array([1.34, 1.22, -1.538]))

      hand_joints = self.interpolate_pos(step, self.normalize_hand_joints(np.array([0.025, 0.025])), self.normalize_hand_joints(np.array([0.00,0.00])), 0, 1) 

    # leave pan
    elif phase_end_step[18] < step <= phase_end_step[19]:
      step = (step- phase_end_step[18]) / phase_lenth[19]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.2235833, 0.753, -0.29894835, 1.8524919, 0.14503075, 1.165, -0.064994514]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([1.34, 1.22, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([2.629, 1.22, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      if step == 0.9:
        list = self.euclidean_distance(mobile_prev_joints,mobile_next_joints)
        self.mobile_dist_list.append(list)

      hand_joints = self.normalize_hand_joints(np.array([0.00, 0.00]))
      
    # put down the pan
    elif phase_end_step[19] < step <= phase_end_step[20]:
      step = (step- phase_end_step[19]) / phase_lenth[20]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.21475278, 0.87421185, -0.29423642, 2.1765423, 0.14786041, 1.5147234, -0.06511537]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([2.629, 1.22, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([2.629, 1.357, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.00, 0.00]))
      
    # back to the init
    elif phase_end_step[20] < step <= phase_end_step[21]:
      step = (step- phase_end_step[20]) / phase_lenth[21]
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.21475278, 0.87421185, -0.29423642, 2.1765423, 0.14786041, 1.5147234, -0.06511537]))
      arm_next_joints = self.normalize_arm_joints(np.array([-0.022, 0.011, 0.022, 1.577, 0.022, 1.563, 0.065]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_joints = self.normalize_mobile_joints(np.array([2.629, 1.357, -1.538]))

      hand_joints = self.normalize_hand_joints(np.array([0.040,0.040]))

    else:
      print("Wrong step")
      raise NotImplementedError
    
    if control_mode_index == 1:
      arm_pose = np.zeros(7)
      arm_pose[0:3], arm_pose[3:7] = arm_trans, arm_rot
    elif control_mode_index == 2:
      arm_pose = np.zeros(6)
      arm_pose[0:3], arm_pose[3:6] = arm_trans, arm_rot
    elif control_mode_index == 0:
      arm_pose = np.zeros(self.arm_dof)
      arm_pose[0:self.arm_dof] = arm_joints
      
    else:
      raise NotImplementedError
    
    key_point_info = dict(
        arm_pose=arm_pose,
        hand_joints= hand_joints,
        mobile_joints = mobile_joints,
        control_mode_index= control_mode_index
      )
    
    return key_point_info

  def euclidean_distance(self, array1, array2):
  
      diff = array1 - array2
      squared_diff = diff ** 2
      sum_squared_diff = np.sum(squared_diff, axis=0)
      euclidean_distance = np.sqrt(sum_squared_diff)
      
      return euclidean_distance

  def normalize_list(self, input_list):
      """
      scale size the input list to [0, 1]
      parameters：
      - input_list (list): need normalized list。
      returns：
      - normalized_list (list): after normalized list
      """
      array = np.array(input_list)
      normalized_array = (array - array.min()) / (array.max() - array.min())
      normalized_list = normalized_array.tolist()

      return normalized_list

  def pick_banana_pan(self, robot:sapien.ArticulationBase, step):
    if (step-5) % self.time_interval == 0 :
      self._cur_base_qpos = self.normalize_mobile_joints(np.array(robot.get_qpos()[0: self.mobile_dof]))
      self._cur_arm_qpos = self.normalize_arm_joints(np.array(robot.get_qpos()[self.mobile_dof: self.mobile_dof+self.arm_dof]))
      self._cur_hand_qpos = self.normalize_hand_joints(np.array(robot.get_qpos()[self.mobile_dof+self.arm_dof:]))
      self._cur_ee_pose = [link for link in robot.get_links() if link.get_name() == 'xarm_gripper_base_link'][0].get_pose()
    
    self._hand_init_joints = self.normalize_hand_joints(np.array([0.044,0.044])) 

    # init
    if 5 <= step <= 5 + 1.0 * self.time_interval:
      control_mode_index = 0
      step = (step-5) / (1 * self.time_interval)

      arm_joints = self.normalize_arm_joints(np.array([0.108, 0.639, -0.36713588, 1.86, 0.28100064, 0.816, 0.103429176]))

      mobile_joints = self.normalize_mobile_joints(np.array([1.065, 1.093, -1.571]))

      hand_joints = self.normalize_hand_joints(np.array([0.040,0.040])) 

    # close to pick banana
    elif 5 + 1 * self.time_interval < step <= 5 + 2.0 * self.time_interval:
      step = (step-(5 + 1 * self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 1

      arm_trans = self.interpolate_pos(step,self._cur_ee_pose.p,np.array([-0.837,0.794,0.263]),0,1)
      arm_rot = self.slerp_quat(step,self._cur_ee_pose.q,np.array([0.093,-0.090,-0.992,-0.004]),0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([1.065, 1.093, -1.571]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([1.065, 1.045, -1.571]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.interpolate_pos(step, self._cur_hand_qpos, self.normalize_hand_joints(np.array([0.030,0.030])), 0, 1) 

    # lift banana
    elif 5 + 2.0 *  self.time_interval < step <= 5 + 3.0 * self.time_interval:
      print("=== Phase: 3 ===")

      step = (step-(5 + 2.0 *  self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 0

      arm_prev_joints = self._cur_arm_qpos
      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_joints = self._cur_base_qpos

      hand_joints =self.normalize_hand_joints(np.array([0.015, 0.015]))

    # go to the target place
    elif 5 + 3.0 *  self.time_interval < step <= 5 + 3.5 * self.time_interval:
      print("=== Phase: 4 ===")
      step = (step-(5 + 3.0 *  self.time_interval)) / (0.5 * self.time_interval)

      control_mode_index = 0

      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints =  arm_next_joints # self.interpolate_pos(step,self._cur_arm_qpos,arm_next_joints,0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([1.065, 1.093, -1.571]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([3.247, 1.048, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.015, 0.015]))

    # put down banana
    elif 5 + 3.5 * self.time_interval < step <= 5 + 4.0 * self.time_interval:
      print("=== Phase: 5 ===")
      step = (step-(5 + 3.5 * self.time_interval)) / (0.5 * self.time_interval)
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.40829623, 1.081, -0.40938967, 2.441, 0.324, 1.364, -0.065104134]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_joints = self.normalize_mobile_joints(np.array([3.247, 1.048, -1.538]))

      hand_joints = self.interpolate_pos(step, self.normalize_hand_joints(np.array([0.015, 0.015])), self.normalize_hand_joints(np.array([0.020,0.020])), 0, 1) 

    # go to the pan
    elif 5 + 4.0 * self.time_interval < step <= 5 + 4.5 * self.time_interval:
      print("=== Phase: 6 ===")
      step = (step-(5 + 4.0 * self.time_interval)) / (0.5 * self.time_interval)
      control_mode_index = 0
  
      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints =  arm_next_joints # self.interpolate_pos(step,self._cur_arm_qpos,arm_next_joints,0,1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([3.247, 1.048, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([1.323, 1.048, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.044, 0.044]))

   
    # close to the pan
    elif 5 + 4.5 * self.time_interval < step <= 5 + 5.0 * self.time_interval:
      print("=== Phase: 7 ===")
      step = (step-(5 + 4.5 * self.time_interval)) / (0.5 * self.time_interval)
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.194, 0.967, -0.281, 2.243, 0.151, 1.513, -0.06499768]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([1.323, 1.048, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([1.323, 1.22, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.022, 0.035]))

    # catch the pan and leave
    elif 5 + 5.0 * self.time_interval < step <= 5 + 6.0 * self.time_interval:
      print("=== Phase: 9 ===")
      step = (step-(5 + 5.0 * self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.194, 0.967, -0.281, 2.243, 0.151, 1.513, -0.06499768]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([1.323, 1.22, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([2.629, 1.22, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 
      hand_joints = self.normalize_hand_joints(np.array([0.001, 0.001]))
      
    # close door
    elif 5 + 6.0 * self.time_interval < step <= 5 + 7.0 * self.time_interval:
      print("=== Phase: 10 ===")
      step = (step-(5 + 6.0 * self.time_interval)) / (1.0 * self.time_interval)
      control_mode_index = 0

      arm_prev_joints = self.normalize_arm_joints(np.array([0.41, 3.7592923e-05, -0.41, 1.549, 0.108, 1.53, -0.065]))
      arm_next_joints = self.normalize_arm_joints(np.array([0.194, 0.967, -0.281, 2.243, 0.151, 1.513, -0.06499768]))
      arm_joints = self.interpolate_pos(step, arm_prev_joints, arm_next_joints, 0, 1)

      mobile_prev_joints = self.normalize_mobile_joints(np.array([2.629, 1.22, -1.538]))
      mobile_next_joints = self.normalize_mobile_joints(np.array([2.629, 1.357, -1.538]))
      mobile_joints = self.interpolate_pos(step, mobile_prev_joints, mobile_next_joints, 0, 1) 

      hand_joints = self.normalize_hand_joints(np.array([0.001, 0.001]))
      
    # move to the place 
    elif 5 + 7.0 * self.time_interval < step <= 5 + 8.0 * self.time_interval:
      print("=== Phase: 11 ===")
      step = (step-(5 + 7.0 * self.time_interval)) / (1.0* self.time_interval)
      control_mode_index = 0

      arm_joints = self.normalize_arm_joints(np.array([-0.022, 0.011, 0.022, 1.577, 0.022, 1.563, 0.065]))

      mobile_joints = self.normalize_mobile_joints(np.array([2.629, 1.357, -1.538]))

      hand_joints = self.normalize_hand_joints(np.array([0.040,0.040])) 

    else:
      print("Wrong step")
      raise NotImplementedError
    
    if control_mode_index == 1:
      arm_pose = np.zeros(7)
      arm_pose[0:3], arm_pose[3:7] = arm_trans, arm_rot
    elif control_mode_index == 2:
      arm_pose = np.zeros(6)
      arm_pose[0:3], arm_pose[3:6] = arm_trans, arm_rot
    elif control_mode_index == 0:
      arm_pose = np.zeros(self.arm_dof)
      arm_pose[0:self.arm_dof] = arm_joints
      
    else:
      raise NotImplementedError
    
    key_point_info = dict(
        arm_pose=arm_pose,
        hand_joints= hand_joints,
        mobile_joints = mobile_joints,
        control_mode_index= control_mode_index
      )
    
    return key_point_info
  
  def interpolate_pos(self, step, pos_start, pos_end, t_start, t_end):
      return pos_start + (pos_end - pos_start) * ((step - t_start) / (t_end - t_start))

  def interpolate_quat(self, step, quat_start, quat_end, t_start, t_end):
      euler_start = R.from_quat(quat_start).as_euler('xyz')
      euler_end = R.from_quat(quat_end).as_euler('xyz')
      euler_interpolated = self.interpolate_pos(step, euler_start, euler_end, t_start, t_end)
      return R.from_euler('xyz', euler_interpolated).as_quat()

  def slerp_quat(self, step, quat_start, quat_end, t_start, t_end):
      # Normalize the quaternions just to be safe
      quat_start = quat_start / np.linalg.norm(quat_start)
      quat_end = quat_end / np.linalg.norm(quat_end)

      # Compute the cosine of the angle between the two vectors.
      dot = np.dot(quat_start, quat_end)
      if dot < 0.0:
          quat_end = -quat_end
          dot = -dot

      DOT_THRESHOLD = 0.9995
      if dot > DOT_THRESHOLD:
          result = quat_start + step * (quat_end - quat_start)
          return result / np.linalg.norm(result)

      # Since dot is in range [0, DOT_THRESHOLD], acos is safe
      theta_0 = np.arccos(dot)  # theta_0 = angle between input vectors
      theta = theta_0 * step  # theta = angle between v0 and result
      sin_theta = np.sin(theta)  # compute this value only once
      sin_theta_0 = np.sin(theta_0)  # compute this value only once

      s0 = np.cos(theta) - dot * sin_theta / sin_theta_0  # == sin(theta_0 - theta) / sin(theta_0)
      s1 = sin_theta / sin_theta_0
      return (s0 * quat_start) + (s1 * quat_end)


  def normalize_mobile_joints(self, arm_joints):
      qlimit_scope = self._qlimit_scope[0 : self.mobile_dof]
      normalized_joints = np.zeros_like(arm_joints)
      for i, (q_min, q_max) in enumerate(qlimit_scope):
          normalized_joints[i] = 2 * (arm_joints[i] - q_min) / (q_max - q_min) - 1
      return normalized_joints
  
  def normalize_hand_joints(self, hand_joints):
      qlimit_scope = self._qlimit_scope[self.mobile_dof+self.arm_dof:]
      normalized_joints = np.zeros_like(hand_joints)
      for i, (q_min, q_max) in enumerate(qlimit_scope):
          normalized_joints[i] = 2 * (hand_joints[i] - q_min) / (q_max - q_min) - 1
      return normalized_joints

  def normalize_arm_joints(self, arm_joints):
      qlimit_scope = self._qlimit_scope[self.mobile_dof: self.mobile_dof+self.arm_dof]
      normalized_joints = np.zeros_like(arm_joints)
      for i, (q_min, q_max) in enumerate(qlimit_scope):
          normalized_joints[i] = 2 * (arm_joints[i] - q_min) / (q_max - q_min) - 1
      return normalized_joints
  