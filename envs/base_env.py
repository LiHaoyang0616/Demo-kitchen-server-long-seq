import numpy as np
import sapien.core as sapien
from sapien.utils import Viewer
from PIL import Image

from utils.trajectory_utils import Trajectory
from utils.controller_utils import set_up_controller
from utils.robot_utils import load_robot, generate_robot_info


class BaseEnv():
    def __init__(self, arm_name='xarm7', control_mode='pd_joint_pos', time_interval = 10):
        self._init_engine_renderer()
        self._init_scene(arm_name, control_mode)
        self._init_trajectory(arm_name, time_interval)
        self._init_viewer()
        self._init_camera()

    def _init_engine_renderer(self):
        self._engine = sapien.Engine()
        self._renderer = sapien.SapienRenderer() # offscreen_only=True,device='cuda:0'
        self._engine.set_renderer(self._renderer)
        self._engine.set_log_level("error")
        sapien.render_config.camera_shader_dir = "rt"
        # sapien.render_config.viewer_shader_dir = "rt"
        sapien.render_config.rt_samples_per_pixel = 64
        sapien.render_config.rt_use_denoiser = True

    def _init_scene(self, arm_name, control_mode):
        self._simulation_freq = 500
        self._scene = self._engine.create_scene()
        scene_config = self._scene.get_config()
        scene_config.default_dynamic_friction = 1.0
        scene_config.default_static_friction = 1.0
        scene_config.default_restitution = 0.0
        scene_config.contact_offset = 0.02
        scene_config.enable_pcm = False
        scene_config.solver_iterations = 100

        self._scene.set_timestep(1 / self._simulation_freq)  # Simulate in 500Hz
        self._add_background()
        # self._add_table()
        self._add_agent(arm_name, control_mode)
        self._add_workspace()
        self._add_actor()

    def _init_trajectory(self, arm_name, time_interval):
        self.traj = Trajectory(arm_name, time_interval, qlimit_scope = self._qlimit_scope)
        
    def _add_background(self):
        physical_material = self._scene.create_physical_material(1.0, 1.0, 0.0)
        self._scene.default_physical_material = physical_material
        render_material = self._renderer.create_material()
        render_material.set_base_color([0.21, 0.18, 0.14, 1.0]) # type: ignore
        render_material.set_diffuse_texture_from_file('./assets/kitchen/new_kitchen/Wood_Floor_footprints_Texture.jpg') # type: ignore
        # render_material.set_roughness_texture_from_file('./assets/kitchen/new_kitchen/wood-seam-pattern.png')
        self._scene.add_ground(render_material=render_material, altitude = -1.0, render_half_size=[6,6]) # type: ignore
        # self._scene.set_environment_map_from_files(px='./assets/kitchen/Standard-Cube-Map/px.png',py='./assets/kitchen/Standard-Cube-Map/py.png',pz='./assets/kitchen/Standard-Cube-Map/pz.png',nx='./assets/kitchen/Standard-Cube-Map/nx.png',ny='./assets/kitchen/Standard-Cube-Map/ny.png',nz='./assets/kitchen/Standard-Cube-Map/nz.png')
        self._scene.set_ambient_light([0.6, 0.6, 0.6]) # type: ignore
        self._scene.add_directional_light([1, 1, 0], [1.0, 1.0, 1.0], position=[0, 0, 0.883], far=10, shadow=False, scale=10, shadow_map_size=2048) # type: ignore
        self._scene.add_directional_light([1, 0.5, -1.6], [1.1, 1.1, 1.1], position=[0, 0, 0.883], far=10, shadow=True, scale=10, shadow_map_size=2048) # type: ignore

    def _add_table(self, pose=sapien.Pose(p=[-0.05-0.2, 0.2, 0]), length=0.5, width=1.83, height=0.97, thickness=0.03, color=(0.8, 0.6, 0.4), name='table'): # type: ignore
        builder = self._scene.create_actor_builder()
        # Tabletop
        tabletop_pose = sapien.Pose([0., 0., -thickness / 2])  # type: ignore # Make the top surface's z equal to 0
        tabletop_half_size = [length / 2, width / 2, thickness / 2]
        builder.add_box_collision(pose=tabletop_pose, half_size=tabletop_half_size) # type: ignore
        # builder.add_box_visual(pose=tabletop_pose, half_size=tabletop_half_size, color=color) # type: ignore
        # Table legs (x4)
        for i in [-1, 1]:
            for j in [-1, 1]:
                x = i * (length - thickness) / 2
                y = j * (width - thickness) / 2
                table_leg_pose = sapien.Pose([x, y, -height / 2]) # type: ignore
                table_leg_half_size = [thickness / 2, thickness / 2, height / 2]
                builder.add_box_collision(pose=table_leg_pose, half_size=table_leg_half_size) # type: ignore
                # builder.add_box_visual(pose=table_leg_pose, half_size=table_leg_half_size, color=color) # type: ignore
        table = builder.build_static(name=name)
        table.set_pose(pose)
        self.table = table

    def _add_agent(self, arm_name, control_mode, x_offset= 0.029 , y_offset=-0.150, z_offset=0):
        """
            Initialize control property, build robots and set up controllers.
        """
        self._init_control_property(control_mode=control_mode)   # initialize control property before adding robots.
        # NOTE(chichu): allegro hands used here have longer customized finger tips
        # TODO(chichu): add ability hand.
        self.arm_dof, self.hand_dof, self.mobile_dof = [7], [2], [3]
        self.robot_name = ['xarm7_reduced_gripper_mobile']
        self.robot_left = load_robot(self._scene, "xarm7_reduced_gripper_mobile")
        self.robot_left.set_root_pose(sapien.Pose([x_offset, -y_offset+0.2, z_offset], [0, 0, 0, 1])) # type: ignore
        self.controller_robot_left = set_up_controller(arm_name=arm_name, control_mode=self._control_mode, robot=self.robot_left)
        self.robot = [self.robot_left]
        self.controller = [self.controller_robot_left]
        self._qlimit_scope = self.robot[0].get_qlimits()
        print("self._qlimit_scope",self._qlimit_scope)
        self._time_interval = 10
        # self._init_cache_robot_info()

    def _add_workspace(self):
        """ Add workspace.
        """
        raise NotImplementedError

    def _add_actor(self):
        """ Add actors
        """
        raise NotImplementedError

    def _init_viewer(self):
        self.viewer = Viewer(self._renderer)
        self.viewer.set_scene(self._scene)
        self.viewer.set_camera_xyz(x=2.5, y=1.0, z=1.0)
        self.viewer.set_camera_rpy(r=0, p=-0.0, y = -3.0)

        # camera = self._scene.add_camera(name="main_camera", width=1024, height=768, fovy=0.96, near=0.1, far=100)
        # camera.set_local_pose(sapien.Pose([-2.80045, -2.1693, 1.84094], [0.941692, -0.0546976, 0.188441, 0.273338]))


    def _init_camera(self):
        self.camera_mount = self._scene.create_actor_builder().build_kinematic()
        self.camera = self._scene.add_mounted_camera(
                name="main_camera1",
                actor=self.camera_mount,
                pose=sapien.Pose(),  # relative to the mounted actor
                width=1280,
                height=720,
                fovy=np.deg2rad(45),
                near=0.1,
                far=100,
            )
        
        self.camera_mount_fridge = self._scene.create_actor_builder().build_kinematic()
        self.camera_fidge = self._scene.add_mounted_camera(
                name="camera_fidge", 
                actor=self.camera_mount_fridge,
                pose=sapien.Pose(),  # relative to the mounted actor
                width=1808, 
                height=1268, 
                fovy=1.05, 
                near=0.1, 
                far=100)
        
        self.camera_mount_teapot = self._scene.create_actor_builder().build_kinematic()
        self.camera_teapot = self._scene.add_mounted_camera(
                name="camera_teapot", 
                actor=self.camera_mount_teapot,
                pose=sapien.Pose(),  # relative to the mounted actor
                width=1808, 
                height=1268, 
                fovy=1.06, 
                near=0.1, 
                far=100)

        self.camera_mount.set_pose(sapien.Pose([-2.80045, -2.1693, 1.84094], [0.941692, -0.0546976, 0.188441, 0.273338]))
        self.camera_mount_fridge.set_pose(sapien.Pose([0.841457, -2.80656, 1.00341], [0.521911, -0.122342, 0.0759455, 0.840758]))
        self.camera_mount_teapot.set_pose(sapien.Pose([-0.485328, 1.84569, 0.976423], [0.86458, 0.0600084, 0.106444, -0.487412]))
        
    def _init_control_property(self, control_freq=20, control_mode='pd_joint_pos'):
        """
            Initialize basic control propert
            NOTE(chichu): pid gains are set in load_robot() function.
        """
        self._control_mode = control_mode
        self._control_freq = control_freq
        assert (self._simulation_freq % self._control_freq == 0)
        self._frame_skip = self._simulation_freq // self._control_freq
        self._control_time_step = 1 / self._control_freq
        
    def reset(self):
        # Set robot initial qpos
        for index in range(len(self.robot)):
            qpos = np.zeros(self.robot[index].dof)
            xarm_qpos = np.array([0, 0, 0, 0, 0, -np.pi / 2, 0])
            qpos[:self.arm_dof[index]] = xarm_qpos
            self.robot[index].set_qpos(qpos)
            self.robot[index].set_drive_target(qpos)
        # Reset controller
        for index in range(len(self.controller)):
            self.controller[index].reset()

    def step(self, action):
        self.step_action(action)

    def step_action(self, action):
        if action is not None:
            self._before_control_step()
            self._set_target(action)
        for _ in range(self._frame_skip): # self._frame_skip
            self._before_simulation_step()
            self._simulation_step()
            self._after_simulation_step()
        self._after_control_step()


    def get_ee_pose(self):
        for i in range(len(self.robot)):
            self.ee_pose[i] = [link for link in self.robot[i].get_links() if link.get_name() == self.ee_link_name][0].get_pose()

        return self.ee_pose
    
    def switch_control_mode(self, robot:sapien.Articulation, arm_name, control_mode):
        self._control_mode = control_mode
        self.controller_robot_left = set_up_controller(arm_name=arm_name,control_mode=self._control_mode, robot=self.robot_left)
        self.controller = [self.controller_robot_left]

    def _before_control_step(self):
        return
        for index in range(len(self.robot)):
            self.current_qpos[index] = self.robot[index].get_qpos()

    def _set_target(self, action):
        self.controller[0].set_action(action)
        return None

    def _after_control_step(self):
        # pass
        self._scene.update_render()
        # no windows
        self.viewer.render()

    def _before_simulation_step(self):
        for robot in self.robot:
            passive_qf = robot.compute_passive_force(external=False)
            robot.set_qf(passive_qf)

    def _simulation_step(self):
        self._scene.step()
    
    def _after_simulation_step(self):
        pass

    def _init_cache_robot_info(self, root_frame='robot'):
        self.robot_info = generate_robot_info()
        self.arm_dof, self.hand_dof, self.mobile_dof = [], [], []
        self.ee_link_name = 'base_link'
        self.ee_pose = []
        for i, name in enumerate(self.robot_name):
            self.arm_dof.append(self.robot_info[name].arm_dof)
            self.hand_dof.append(self.robot_info[name].hand_dof)
            self.mobile_dof.append(self.robot_info[name].mobile_dof)
            self.ee_pose.append([link for link in self.robot[i].get_links() if link.get_name() == self.ee_link_name][0].get_pose())
    
    def set_time_interval(self, time_interval):
        self._time_interval = time_interval
    
    # ---------------------------------------------------------------------------- #
    # Utilities
    # ---------------------------------------------------------------------------- #
    def get_actor(self, name) -> sapien.ActorBase:
        all_actors = self._scene.get_all_actors()
        actor = [x for x in all_actors if x.name == name]
        if len(actor) > 1:
            raise RuntimeError(f'Not a unique name for actor: {name}')
        elif len(actor) == 0:
            raise RuntimeError(f'Actor not found: {name}')
        return actor[0]

    def get_dynamic_actor(self, name) -> sapien.ActorDynamicBase:
        all_actors = self._scene.get_all_actors()
        actor = [x for x in all_actors if x.name == name]
        if len(actor) > 1:
            raise RuntimeError(f'Not a unique name for actor: {name}')
        elif len(actor) == 0:
            raise RuntimeError(f'Actor not found: {name}')
        return actor[0]

    def get_articulation(self, name) -> sapien.ArticulationBase:
        all_articulations = self._scene.get_all_articulations()
        articulation = [x for x in all_articulations if x.name == name]
        if len(articulation) > 1:
            raise RuntimeError(f'Not a unique name for articulation: {name}')
        elif len(articulation) == 0:
            raise RuntimeError(f'Articulation not found: {name}')
        return articulation[0]