from .base_env import BaseEnv
import sapien.core as sapien


class OpenFridge(BaseEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.banana = self.get_actor('banana')

    def _add_workspace(self):
        # add kitchen_envs
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(filename='./assets/kitchen/new_kitchen/newkitchen2.glb')
        # builder.add_multiple_collisions_from_file(scale=[1.0,1.0,0.955], filename='./assets/outputs/kitchen1.obj')
        kitchen_env = builder.build_kinematic(name='kitchen_env') # can not be affected by external forces
        kitchen_env.set_pose(sapien.Pose(p=[-0.957, 0.704, -0.770],q=[0, 0, -0.707, -0.707]))

        # add left_table 
        self._add_table(
            pose=sapien.Pose(p=[-1.293, -1.460, 0.102],q=[1, 0, 0, 0]),
            length=1.6,
            width=1.8, 
            height=0.87, 
            thickness=0.03, 
            color=(0.8, 0.6, 0.4), 
            name='long_table')

        # add sink_table 
        self._add_table(
            pose=sapien.Pose(p=[-0.911, -0.219, -0.170],q=[1, 0, 0, 0]),
            length=0.6,
            width=0.6, 
            height=0.57, 
            thickness=0.03, 
            color=(0.8, 0.6, 0.4), 
            name='sink_table')

        # add right_table 
        self._add_table(
            pose=sapien.Pose(p=[-1.293, 0.593, 0.102],q=[1, 0, 0, 0]),
            length=1.6,
            width=0.9, 
            height=0.87, 
            thickness=0.03, 
            color=(0.8, 0.6, 0.4), 
            name='right_table')
        
        # add small_table 
        self._add_table(
            pose=sapien.Pose(p=[-1.657, -0.205, 0.102],q=[1, 0, 0, 0]),
            length=0.88,
            width=0.68, 
            height=0.87, 
            thickness=0.03, 
            color=(0.8, 0.6, 0.4), 
            name='small_table')

        # add long_table 
        self._add_table(
            pose=sapien.Pose(p=[1.489, -0.481, 0.102],q=[1, 0, 0, 0]),
            length=0.6,
            width=4.88, 
            height=0.87, 
            thickness=0.03, 
            color=(0.8, 0.6, 0.4), 
            name='long_table')
        
        # add sink
        builder = self._scene.create_actor_builder()
        builder.add_multiple_collisions_from_file(scale=[1.5,2.0,1.4], filename='./assets/kitchen/sink/stainless_steel_sink.obj')
        sink = builder.build_static(name='sink') # can not be affected by external forces
        sink.set_pose(sapien.Pose(p=[-0.935, -0.233, -0.058],q=[0.707, 0, 0, -0.707]))

    def _add_actor(self):
        # add pan2
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(scale=[0.4,0.4,0.4], filename='./assets/kitchen/pan/pan_2.dae')
        # builder.add_multiple_collisions_from_file(scale=[0.4,0.4,0.4], density=500, filename='./assets/kitchen/pan/pan_2.obj')
        builder.add_box_collision(half_size=[0.02, 0.02, 0.01],pose=sapien.Pose(p=[0.0,0.0,0.0]))
        builder.add_box_collision(half_size=[0.08, 0.02, 0.01],pose=sapien.Pose(p=[-0.3,0.0,0.0]))
        pan_2 = builder.build(name = 'pan2') # can not be affected by external forces
        pan_2.set_pose(sapien.Pose(p=[-0.865, 0.336, 0.243],q=[0,0,0.707,0.707]))

        # add teapot
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(scale=[0.542,0.542,0.542],filename='./assets/kitchen/teacup/teapot.dae')
        # builder.add_multiple_collisions_from_file(filename='./assets/kitchen/teapot/teapot.obj')
        PhysicalMaterial = self._scene.create_physical_material(0.9,0.9,0.0)
        PhysicalMaterial.set_static_friction(0.999)
        PhysicalMaterial.set_dynamic_friction(0.999)
        builder.add_box_collision(density=1000,half_size=[0.045, 0.008, 0.018],pose=sapien.Pose(p=[0.0,0.218,0.0]))
        builder.add_box_collision(material = PhysicalMaterial, density=500,half_size=[0.07, 0.01, 0.07],pose=sapien.Pose(p=[0.0,0.0,0.0]))
        teapot = builder.build_static(name = 'teapot') # can not be affected by external forces
        teapot.set_pose(sapien.Pose(p=[1.438, 0.612, 0.112],q=[-0.498,-0.498,0.502,0.502]))

        # add tea_cup
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(scale=[0.542,0.542,0.542],filename='./assets/kitchen/teacup/teacup.dae')
        builder.add_box_collision(half_size=[0.14, 0.0001, 0.3])
        tea_cup = builder.build(name = 'tea_cup') # can not be affected by external forces
        tea_cup.set_pose(sapien.Pose(p=[1.417, 0.464, 0.118],q=[0, 0, 0.707, 0.707]))

        # add utensil
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(scale=[1.542,1.542,1.542],filename='./assets/kitchen/utensil/utensil.dae')
        builder.add_box_collision(half_size=[0.1, 0.01, 0.1])
        utensil = builder.build(name = 'utensil') # can not be affected by external forces
        utensil.set_pose(sapien.Pose(p=[1.348, -2.333, 0.119],q=[0.707, 0.707, 0.0, 0.0]))

        # add coffee_set
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(filename='./assets/kitchen/coffee/Coffee_shop_set.glb')
        builder.add_box_collision(half_size=[0.1, 0.01, 0.1])
        coffee_set = builder.build(name = 'coffee_set') # can not be affected by external forces
        coffee_set.set_pose(sapien.Pose(p=[1.633, -0.059, 0.148],q=[0.473,0.529,-0.530,-0.464]))

        # add coffee
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(filename='./assets/kitchen/coffee/coffee.glb')
        builder.add_box_collision(half_size=[0.1, 0.01, 0.1])
        coffee = builder.build(name = 'coffee') # can not be affected by external forces
        coffee.set_pose(sapien.Pose(p=[1.540, -2.070, 0.112],q=[0.507,0.507,-0.493,-0.493]))

        # add vase
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(scale=[2.4,2.4,2.4],filename='./assets/kitchen/branches_vase/vase.dae')
        builder.add_box_collision(half_size=[0.1, 0.01, 0.1])
        vase = builder.build(name = 'vase') # can not be affected by external forces
        vase.set_pose(sapien.Pose(p=[-1.876, -0.284, 0.112],q=[0.471, 0.471, 0.528, 0.528]))

        # add knife
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(scale=[0.8,0.8,0.8],filename='./assets/kitchen/knife/knife.dae')
        builder.add_box_collision(half_size=[0.03, 0.03, 0.01])
        knife = builder.build_static(name = 'knife') # can not be affected by external forces
        knife.set_pose(sapien.Pose(p=[-0.633,-1.438,0.112],q=[-0.905,0.0,0.0,-0.426])) # allego: p=[-0.795, 0.632, -0.036],q=[-0.707, 0, 0, -0.707])
    
        # add boards
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(filename='./assets/kitchen/vegetable/board.dae')
        builder.add_multiple_collisions_from_file(filename='./assets/kitchen/vegetable/board.obj')
        boards = builder.build_static(name = 'boards') # can not be affected by external forces
        boards.set_pose(sapien.Pose(p=[-0.627, -1.7, 0.102],q=[-0.495, -0.492, 0.505, 0.507]))

        # add shelf
        builder = self._scene.create_actor_builder()
        builder.add_box_visual(half_size=[0.29, 0.25, 0.01], color=[0.8, 0.8, 0.8])
        builder.add_box_collision(half_size=[0.29, 0.25, 0.01])
        shelf = builder.build_static(name = 'shelf') # can not be affected by external forces
        shelf.set_pose(sapien.Pose(p=[-0.302, 2.380, 0.009],q=[0,0,1,0]))

        # add fruits
        builder = self._scene.create_actor_builder()
        builder.add_visual_from_file(scale=[0.03,0.03,0.03],filename='./assets/kitchen/refrigerator/fruits.glb')
        # builder.add_multiple_collisions_from_file(scale=[0.03,0.03,0.03],filename='./assets/kitchen/refrigerator/fruits.obj')
        fruits = builder.build_static(name = 'fruits') # can not be affected by external forces
        fruits.set_pose(sapien.Pose(p=[-0.426, 2.422, 0.006],q=[-0.521,-0.518,0.478,0.480]))

        # add banana
        builder = self._scene.create_actor_builder()
        PhysicalMaterial = self._scene.create_physical_material(2.0,2.0,0.0)

        builder.add_visual_from_file(pose=sapien.Pose(p=[-0.025,0.005,-0.02],q=[0.707,0,-0.707,0]),scale=[0.3,0.3,0.3],filename='./assets/kitchen/vegetable/zucchini01.glb')
        builder.add_multiple_collisions_from_file(material=PhysicalMaterial, scale=[0.03,0.03,0.03],filename='./assets/kitchen/refrigerator/banana.obj') # add_box_collision(material=PhysicalMaterial, density=100, pose=sapien.Pose(p=[-0.0215,0.01,0.0]), half_size=[0.018,0.020,0.065]) # 
        banana = builder.build(name = 'banana') # can not be affected by external forces
        self.banana = banana
        banana.set_pose(sapien.Pose(p=[-0.377, 2.232, 0.030],q=[0.343, -0.723, -0.172, 0.575])) # open_fridge

        # add refrigerator:     
        loader: sapien.URDFLoader = self._scene.create_urdf_loader()
        loader.fix_root_link = True     # fix_root_link
        robotbuild: sapien.ArticulationBuilder = loader.load_file_as_articulation_builder("./assets/kitchen/refrigerator/10347/mobility.urdf")
        refrigerator = robotbuild.build(fix_root_link=True)
        refrigerator.set_name('refrigerator')
        refrigerator.set_root_pose(sapien.Pose([-0.305, 2.321, 0.195], [0.707, 0, 0, 0.707]))
        refrigerator.set_qpos([0.0, 0.0])

    def get_actor_pose(self, actor_name):
        # TODO(haoyang): get actor's pose of actor_name. 
        return None