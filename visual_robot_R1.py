import sapien
import mplib.planner
import os
import sys
import numpy as np
# from envs.utils.create_actor import create_actor
# 获取 RoboTwin 目录（envs/utils 的上级）
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
sys.path.insert(0, project_root)

scene = sapien.Scene()
scene.add_ground(0)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

viewer = scene.create_viewer()
viewer.set_camera_xyz(x=-2, y=0, z=1)
viewer.set_camera_rpy(r=0, p=-0.3, y=0)


loader = scene.create_urdf_loader()
loader.fix_root_link = True

robot = loader.load("/home/nemo/maniskill/assets/URDF/R1/urdf/r1_v2_1_0.urdf")
robot.set_root_pose(sapien.Pose([0, -0.65, 0], [1, 0, 0, 1]))

qpos = np.zeros(26)

# --------- 底盘与转向相关关节 ---------
qpos[0]  = 0.0  # steer_motor_joint1
qpos[1]  = 0.0  # steer_motor_joint2
qpos[2]  = 0.0  # steer_motor_joint3

# --------- torso 主体关节（升降/旋转）---------
qpos[3]  = 0.0  # torso_joint1
qpos[4]  = 0.0  # wheel_motor_joint1
qpos[5]  = 0.0  # wheel_motor_joint2
qpos[6]  = 0.0  # wheel_motor_joint3
qpos[7]  = 0.0  # torso_joint2
qpos[8]  = 0.0  # torso_joint3
qpos[9] = 0.0  # torso_joint4

# --------- 左臂 6-DoF ---------
qpos[10] = 1.5708  # left_arm_joint1
qpos[12] = 3.0  # left_arm_joint2
qpos[14] = -2.8  # left_arm_joint3
qpos[16] = 0.17  # left_arm_joint4
qpos[18] = 0.023  # left_arm_joint5
qpos[20] = -0.25  # left_arm_joint6

# --------- 右臂 6-DoF ---------
qpos[11] = -1.5708  # right_arm_joint1
qpos[13] = 3.0  # right_arm_joint2
qpos[15] = -2.8  # right_arm_joint3
qpos[17] = 0.17  # right_arm_joint4
qpos[19] = 0.023  # right_arm_joint5
qpos[21] = -0.04  # right_arm_joint6

# --------- 左右夹爪指头 ---------
qpos[22] = 0.0  # left_gripper_finger_joint1
qpos[23] = 0.0  # left_gripper_finger_joint2
qpos[24] = 0.0  # right_gripper_finger_joint1
qpos[25] = 0.0  # right_gripper_finger_joint2

# 应用到机器人
robot.set_qpos(qpos)

active_joints = robot.get_active_joints()
for joint in active_joints:
    joint.set_drive_property(stiffness=0, damping=0.5)
    joint.set_drive_property(stiffness=0, damping=0.5)

def stabilize_joints(robot, joint_names, damping=200.0, force_limit=20.0):
    for name in joint_names:
        joint = robot.find_joint_by_name(name)
        joint.set_drive_property(stiffness=0.0, damping=damping, force_limit=force_limit)
        joint.set_drive_velocity_target(0.0)

wheel_joints = ["wheel_motor_joint1", "wheel_motor_joint2", "wheel_motor_joint3"]
steer_joints = ["steer_motor_joint1", "steer_motor_joint2", "steer_motor_joint3"]
stabilize_joints(robot, wheel_joints)
stabilize_joints(robot, steer_joints)

all_joints = robot.get_joints()
left_endpose = robot.find_joint_by_name("left_arm_joint6")
right_endpose = robot.find_joint_by_name("right_arm_joint6")

# # 创建规划器
# left_planner = mplib.Planner(
#             urdf="/home/nemo/maniskill/assets/URDF/R1/urdf/r1_v2_1_0.urdf",
#             #srdf="/home/pine/RoboTwin/aloha_maniskill_sim/srdf/arx5_description_isaac.srdf",
#             move_group="fl_link6",
#         )
# right_planner = mplib.Planner(
#             urdf="/home/pine/RoboTwin/aloha_maniskill_sim/urdf/arx5_description_isaac.urdf",
#             srdf="/home/pine/RoboTwin/aloha_maniskill_sim/srdf/arx5_description_isaac.srdf",
#             move_group="fr_link6",
#         )
# robot_pose_in_world = [0,-0.65,0,1,0,0,1] 
# left_planner.set_base_pose(robot_pose_in_world)
# right_planner.set_base_pose(robot_pose_in_world)

# left_arm_joint_id = [6,14,18,22,26,30]
# right_arm_joint_id = [7,15,19,23,27,31]


# left_arm_target_qpos = [-0.2687194049358368, 1.7130846977233887, 0.720784068107605, 0.942753255367279, 1.3004862070083618, 0.07230053097009659]   # 示例左臂关节角 -1.422
# right_arm_target_qpos = [0,0,0,0,0,0]  # 示例右臂关节角 # 1.60

# joint_pose = robot.get_qpos()

# # 写入指定关节的值--------------------------------------------------------------------------------
# # 替换指定关节的值
# for i in range(6):
#     joint_pose[left_arm_joint_id[i]] = left_arm_target_qpos[i]
#     joint_pose[right_arm_joint_id[i]] = right_arm_target_qpos[i]

# robot.set_qpos(joint_pose)

# tag = np.random.randint(0,2)
# print("tag,", tag)  

# coaster.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

while not viewer.closed:
    for _ in range(4):  # render every 4 steps
        qf = robot.compute_passive_force(
            gravity=True,
            coriolis_and_centrifugal=True,
        )
        robot.set_qf(qf)
        scene.step()
    scene.update_render()
    viewer.render()
