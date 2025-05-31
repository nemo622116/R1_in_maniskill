import sapien.core as sapien
from mani_skill.envs.scene import ManiSkillScene

scene = ManiSkillScene()

loader = scene.create_urdf_loader()
loader.fix_root_link = True
loader.load_multiple_collisions_from_file = True

# 加载 URDF
robot = loader.load("/home/nemo/maniskill/assets/URDF/R1/urdf/r1_v2_1_0.urdf")

print(f"[INFO] Loaded {len(robot.get_links())} links and {len(robot.get_joints())} joints")

# 打印碰撞形状
for link in robot.get_links():
    try:
        shapes = link.get_collision_shapes()
        print(f"[DEBUG] Link: {link.get_name()}, #collision_shapes: {len(shapes)}")
    except Exception as e:
        print(f"[WARNING] Link: {link.get_name()} raised error: {e}")
