import sapien
from mani_skill.envs.scene import ManiSkillScene
from mani_skill.utils.building import URDFLoader
loader = URDFLoader()
loader.set_scene(ManiSkillScene())
robot = loader.load("/home/nemo/maniskill/assets/URDF/R1/urdf/r1_v2_1_0.urdf")
print(robot.active_joints_map.keys())


