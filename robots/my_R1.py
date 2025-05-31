import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.sensors.camera import CameraConfig


@register_agent()
class MyR1(BaseAgent):
    uid = "my_R1"
    urdf_path = f"/home/nemo/maniskill/assets/URDF/R1/urdf/r1_v2_1_0.urdf"
    fix_root_link = False       #移动机器人固定为False
    qpos = np.zeros(23)

    # --------- base 底盘移动（平移/旋转）---------
    qpos[0]  = 0.0  # root_x_axis_joint
    qpos[1]  = 0.0  # root_y_axis_joint
    qpos[2]  = 0.0  # root_z_rotation_joint

    # --------- torso 主体关节（升降/旋转）---------
    qpos[3]  = 0.0  # torso_joint1
    qpos[4]  = 0.0  # torso_joint2
    qpos[5]  = 0.0  # torso_joint3
    qpos[6]  = 0.0  # torso_joint4

    # --------- 左臂 6-DoF ---------
    qpos[7] = 1.5708  # left_arm_joint1
    qpos[9] = 3.0  # left_arm_joint2
    qpos[11] = -2.8  # left_arm_joint3
    qpos[13] = 0.17  # left_arm_joint4
    qpos[15] = 0.023  # left_arm_joint5
    qpos[17] = -0.25  # left_arm_joint6

    # --------- 右臂 6-DoF ---------
    qpos[8] = -1.5708  # right_arm_joint1
    qpos[10] = 3.0  # right_arm_joint2
    qpos[12] = -2.8  # right_arm_joint3
    qpos[14] = 0.17  # right_arm_joint4
    qpos[16] = 0.023  # right_arm_joint5
    qpos[18] = -0.04  # right_arm_joint6

    # --------- 左右夹爪指头 ---------
    qpos[19] = 0.05  # left_gripper_finger_joint1
    qpos[20] = 0.05  # left_gripper_finger_joint2
    qpos[21] = 0.05  # right_gripper_finger_joint1
    qpos[22] = 0.05  # right_gripper_finger_joint2
    
    keyframes = dict(
        standing=Keyframe(
            qpos=qpos,
            pose=sapien.Pose(p=[0, 0, 0.05], q=[1, 0, 0, 0]),
            qvel=[1 ,1 ,1 , 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        )
    )

    base_joint_names = [
        "root_x_axis_joint", 
        "root_y_axis_joint", 
        "root_z_rotation_joint"
    ]

    torso_joint_names = [
        'torso_joint1', 
        'torso_joint2', 
        'torso_joint3', 
        'torso_joint4'
    ]

    arm_joint_names = [

        'left_arm_joint1', 
        'right_arm_joint1', 
        'left_arm_joint2', 
        'right_arm_joint2', 
        'left_arm_joint3', 
        'right_arm_joint3', 
        'left_arm_joint4', 
        'right_arm_joint4', 
        'left_arm_joint5', 
        'right_arm_joint5', 
        'left_arm_joint6', 
        'right_arm_joint6'
    ]

    gripper_joint_names = [
        'left_gripper_finger_joint1', 
        'left_gripper_finger_joint2', 
        'right_gripper_finger_joint1', 
        'right_gripper_finger_joint2'
    ]
    
    torso_stiffness = 1e3
    torso_damping = 1e2
    torso_force_limit = 100

    arm_stiffness = 1e3
    arm_damping = 1e2
    arm_force_limit = 100

    gripper_stiffness = 1e3
    gripper_damping = 1e2
    gripper_force_limit = 100

    @property
    def _controller_configs(self):
        torso_pd_joint_pos = PDJointPosControllerConfig(
            self.torso_joint_names,
            lower=None,
            upper=None,
            stiffness=self.torso_stiffness,
            damping=self.torso_damping,
            force_limit=self.torso_force_limit,
            normalize_action=False,
        )
        torso_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.torso_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=self.torso_stiffness,
            damping=self.torso_damping,
            force_limit=self.torso_force_limit,
            use_delta=True,
        )
        arm_pd_joint_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=None,
            upper=None,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            normalize_action=False,
        )
        arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            use_delta=True,
        )
        gripper_pd_joint_pos = PDJointPosMimicControllerConfig(
            self.gripper_joint_names,
            lower=-0.01,  # a trick to have force when the object is thin
            upper=0.05,
            stiffness=self.gripper_stiffness,
            damping=self.gripper_damping,
            force_limit=self.gripper_force_limit,
            mimic={
                "left_gripper_finger_joint2": {"joint": "left_gripper_finger_joint1"},
                "right_gripper_finger_joint2": {"joint": "right_gripper_finger_joint1"}
            }
        )
        base_pd_joint_vel = PDBaseVelControllerConfig(
            self.base_joint_names,
            lower=[-0.5, -0.5, -3.14],
            upper=[0.5, 0.5, 3.14],
            damping=1000,
            force_limit=500,
        )

        controller_configs = dict(
            pd_joint_delta_pos=dict(
                torso=torso_pd_joint_delta_pos, 
                arm=arm_pd_joint_delta_pos, 
                gripper=gripper_pd_joint_pos, 
                base=base_pd_joint_vel,
                balance_passive_force=True
            ),
            pd_joint_pos=dict(
                torso=torso_pd_joint_pos, 
                arm=arm_pd_joint_pos, 
                gripper=gripper_pd_joint_pos, 
                base=base_pd_joint_vel,
            ),
        )
        # Make a deepcopy in case users modify any config
        return deepcopy_dict(controller_configs)
    
    @property
    def _sensor_configs(self):
        return [
            CameraConfig(
                uid="head_L515",
                pose=sapien.Pose(p=[0, 0, 0], q=[0.5, -0.5, -0.5, -0.5]),
                width=1024,
                height=768,
                fov=55,
                near=0.01,
                far=9,
                mount=self.robot.links_map["zed_link"]
            ),

            CameraConfig(
                uid="left_camera",
                pose=sapien.Pose(p=[0, 0, 0], q=[0.5, -0.5, -0.5, -0.5]),
                width=1920,
                height=1200,
                fov=55,
                near=0.01,
                far=8,
                mount=self.robot.links_map["left_realsense_link"]
            ),

            CameraConfig(
                uid="right_camera",
                pose=sapien.Pose(p=[0, 0, 0], q=[0.5, -0.5, -0.5, -0.5]),
                width=1920,
                height=1200,
                fov=55,
                near=0.01,
                far=8,
                mount=self.robot.links_map["right_realsense_link"]
            )
        ]
    
    urdf_config = dict(
        _materials=dict(
            gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
        ),
        link=dict(
            left_gripper_finger_link1=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            left_gripper_finger_link2=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            right_gripper_finger_link1=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            right_gripper_finger_link2=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
        ),
    )

    def _after_init(self):
        self.wheel_link_1: Link = self.robot.links_map["wheel_motor_link1"]
        self.wheel_link_2: Link = self.robot.links_map["wheel_motor_link2"]
        self.wheel_link_3: Link = self.robot.links_map["wheel_motor_link3"]
        for link in [self.wheel_link_1, self.wheel_link_2, self.wheel_link_3]:
            link.set_collision_group_bit(group=2, bit_idx=30, bit=1)

    def _load_scene(self, options: dict):
    # ...
        self.ground.set_collision_group_bit(group=2, bit_idx=30, bit=1)
