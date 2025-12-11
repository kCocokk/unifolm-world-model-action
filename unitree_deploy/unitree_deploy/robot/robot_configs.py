import abc
from dataclasses import dataclass, field

import draccus
import numpy as np

from unitree_deploy.robot_devices.arm.configs import (
    ArmConfig,
    G1ArmConfig,
    Z1ArmConfig,
    Z1DualArmConfig,
    D1ArmConfig,
)
from unitree_deploy.robot_devices.cameras.configs import (
    CameraConfig,
    ImageClientCameraConfig,
    IntelRealSenseCameraConfig,
    OpenCVCameraConfig,
)
from unitree_deploy.robot_devices.endeffector.configs import (
    Dex1_GripperConfig,
    EndEffectorConfig,
)

# ======================== arm motors =================================
# name: (index, model)
g1_motors = {
    "kLeftShoulderPitch": [0, "g1-joint"],
    "kLeftShoulderRoll": [1, "g1-joint"],
    "kLeftShoulderYaw": [2, "g1-joint"],
    "kLeftElbow": [3, "g1-joint"],
    "kLeftWristRoll": [4, "g1-joint"],
    "kLeftWristPitch": [5, "g1-joint"],
    "kLeftWristyaw": [6, "g1-joint"],
    "kRightShoulderPitch": [7, "g1-joint"],
    "kRightShoulderRoll": [8, "g1-joint"],
    "kRightShoulderYaw": [9, "g1-joint"],
    "kRightElbow": [10, "g1-joint"],
    "kRightWristRoll": [11, "g1-joint"],
    "kRightWristPitch": [12, "g1-joint"],
    "kRightWristYaw": [13, "g1-joint"],
}

z1_motors = {
    "kWaist": [0, "z1-joint"],
    "kShoulder": [1, "z1-joint"],
    "kElbow": [2, "z1-joint"],
    "kForearmRoll": [3, "z1-joint"],
    "kWristAngle": [4, "z1-joint"],
    "kWristRotate": [5, "z1-joint"],
    "kGripper": [6, "z1-joint"],
}

z1_dual_motors = {
        "kLeftWaist": [0, "z1-joint"],
        "kLeftShoulder": [1, "z1-joint"],
        "kLeftElbow": [2, "z1-joint"],
        "kLeftForearmRoll": [3, "z1-joint"],
        "kLeftWristAngle": [4, "z1-joint"],
        "kLeftWristRotate": [5, "z1-joint"],
        "kRightWaist": [7, "z1-joint"],
        "kRightShoulder": [8, "z1-joint"],
        "kRightElbow": [9, "z1-joint"],
        "kRightForearmRoll": [10, "z1-joint"],
        "kRightWristAngle": [11, "z1-joint"],
        "kRightWristRotate": [12, "z1-joint"],

# D1: 6 关节 + 1 夹爪
d1_motors = {
    "J0": [0, "d1-joint"],
    "J1": [1, "d1-joint"],
    "J2": [2, "d1-joint"],
    "J3": [3, "d1-joint"],
    "J4": [4, "d1-joint"],
    "J5": [5, "d1-joint"],
    "J6": [6, "d1-joint"],  # 夹爪
}}
# =========================================================


# ======================== camera =================================


def z1_intelrealsense_camera_default_factory():
    return {
        "cam_high": IntelRealSenseCameraConfig(
            serial_number="044122071036",
            fps=30,
            width=640,
            height=480,
        ),
        # "cam_wrist": IntelRealSenseCameraConfig(
        #     serial_number="419122270615",
        #     fps=30,
        #     width=640,
        #     height=480,
        # ),
    }


def z1_dual_intelrealsense_camera_default_factory():
    return {
        # "cam_left_wrist": IntelRealSenseCameraConfig(
        #     serial_number="218722271166",
        #     fps=30,
        #     width=640,
        #     height=480,
        # ),
        # "cam_right_wrist": IntelRealSenseCameraConfig(
        #     serial_number="419122270677",
        #     fps=30,
        #     width=640,
        #     height=480,
        # ),
        "cam_high": IntelRealSenseCameraConfig(
            serial_number="947522071393",
            fps=30,
            width=640,
            height=480,
        ),
    }

def d1_intelrealsense_camera_default_factory():
    """D1 使用一只 RealSense 深度相机，挂在机架或末端上方。"""
    return {
        "cam_high": IntelRealSenseCameraConfig(
            serial_number="YOUR_D435_SERIAL",  # 用 rs-enumerate-devices 查到的序列号
            fps=30,
            width=640,
            height=480,
        ),
    }

def g1_image_client_default_factory():
    return {
        "imageclient": ImageClientCameraConfig(
            head_camera_type="opencv",
            head_camera_id_numbers=[4],
            head_camera_image_shape=[480, 1280],  # Head camera resolution
            wrist_camera_type="opencv",
            wrist_camera_id_numbers=[0, 2],
            wrist_camera_image_shape=[480, 640],  # Wrist camera resolution
            aspect_ratio_threshold=2.0,
            fps=30,
            mock=False,
        ),
    }


def usb_camera_default_factory():
    return {
        "cam_high": OpenCVCameraConfig(
            camera_index="/dev/video1",
            fps=30,
            width=640,
            height=480,
        ),
        "cam_left_wrist": OpenCVCameraConfig(
            camera_index="/dev/video5",
            fps=30,
            width=640,
            height=480,
        ),
        "cam_right_wrist": OpenCVCameraConfig(
            camera_index="/dev/video3",
            fps=30,
            width=640,
            height=480,
        ),
    }


# =========================================================


# ======================== endeffector =================================


def dex1_default_factory():
    return {
        "left": Dex1_GripperConfig(
            unit_test=True,
            motors={
                "kLeftGripper": [0, "z1_gripper-joint"],
            },
            topic_gripper_state="rt/dex1/left/state",
            topic_gripper_command="rt/dex1/left/cmd",
        ),
        "right": Dex1_GripperConfig(
            unit_test=True,
            motors={
                "kRightGripper": [1, "z1_gripper-joint"],
            },
            topic_gripper_state="rt/dex1/right/state",
            topic_gripper_command="rt/dex1/right/cmd",
        ),
    }


# =========================================================

# ======================== arm =================================


def z1_arm_default_factory(init_pose=None):
    return {
        "z1": Z1ArmConfig(
            init_pose=np.zeros(7) if init_pose is None else init_pose,
            motors=z1_motors,
        ),
    }


def z1_dual_arm_single_config_factory(init_pose=None):
    return {
        "z1_dual": Z1DualArmConfig(
            left_robot_ip="127.0.0.1",
            left_robot_port1=8073,
            left_robot_port2=8074,
            right_robot_ip="127.0.0.1",
            right_robot_port1=8071,
            right_robot_port2=8072,
            init_pose_left=np.zeros(6) if init_pose is None else init_pose[:6],
            init_pose_right=np.zeros(6) if init_pose is None else init_pose[6:],
            control_dt=1 / 250.0,
            motors=z1_dual_motors,
        ),
    }


def g1_dual_arm_default_factory(init_pose=None):
    return {
        "g1": G1ArmConfig(
            init_pose=np.zeros(14) if init_pose is None else init_pose,
            motors=g1_motors,
            mock=False,
        ),
    }

def d1_arm_default_factory(init_pose=None):
    if init_pose is None:
        init_pose = np.zeros(7, dtype=float)
    init_pose = np.asarray(init_pose, dtype=float).tolist()

    return {
        "d1": D1ArmConfig(
            motors=d1_motors,
            init_pose=init_pose,
            ip="192.168.123.100",  # 按你的实际 D1 IP 改
            topic_command="rt/arm_Command",
            topic_feedback="rt/arm_Feedback",
            topic_servo_angle="current_servo_angle",
        )
    }


# =========================================================


# robot_type:  arm devies _ endeffector devies _ camera devies
@dataclass
class RobotConfig(draccus.ChoiceRegistry, abc.ABC):
    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)


@dataclass
class UnitreeRobotConfig(RobotConfig):
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {})
    arm: dict[str, ArmConfig] = field(default_factory=lambda: {})
    endeffector: dict[str, EndEffectorConfig] = field(default_factory=lambda: {})


# =============================== Single-arm:z1, Camera:Realsense ========================================
@RobotConfig.register_subclass("z1_realsense")
@dataclass
class Z1_Realsense_RobotConfig(UnitreeRobotConfig):
    cameras: dict[str, CameraConfig] = field(default_factory=z1_intelrealsense_camera_default_factory)
    arm: dict[str, ArmConfig] = field(default_factory=z1_arm_default_factory)


# =============================== Dual-arm:z1, Endeffector:dex1, Camera:Realsense ========================================
@RobotConfig.register_subclass("z1_dual_dex1_realsense")
@dataclass
class Z1dual_Dex1_Realsense_RobotConfig(UnitreeRobotConfig):
    cameras: dict[str, CameraConfig] = field(default_factory=z1_dual_intelrealsense_camera_default_factory)
    arm: dict[str, ArmConfig] = field(default_factory=z1_dual_arm_single_config_factory)
    endeffector: dict[str, EndEffectorConfig] = field(default_factory=dex1_default_factory)


# =============================== Dual-arm:z1, Endeffector:dex1, Camera:Realsense ========================================
@RobotConfig.register_subclass("z1_dual_dex1_opencv")
@dataclass
class Z1dual_Dex1_Opencv_RobotConfig(UnitreeRobotConfig):
    cameras: dict[str, CameraConfig] = field(default_factory=usb_camera_default_factory)
    arm: dict[str, ArmConfig] = field(default_factory=z1_dual_arm_single_config_factory)
    endeffector: dict[str, EndEffectorConfig] = field(default_factory=dex1_default_factory)


# =============================== Arm:g1, Endeffector:dex1, Camera:imageclint ========================================
@RobotConfig.register_subclass("g1_dex1")
@dataclass
class G1_Dex1_Imageclint_RobotConfig(UnitreeRobotConfig):
    cameras: dict[str, CameraConfig] = field(default_factory=g1_image_client_default_factory)
    arm: dict[str, ArmConfig] = field(default_factory=g1_dual_arm_default_factory)
    endeffector: dict[str, EndEffectorConfig] = field(default_factory=dex1_default_factory)

@RobotConfig.register_subclass("d1_realsense")
@dataclass
class D1_Realsense_RobotConfig(UnitreeRobotConfig):
    """单臂 D1 + 顶视 RealSense 相机（推荐）。"""

    cameras: dict[str, CameraConfig] = field(default_factory=d1_intelrealsense_camera_default_factory)
    arm: dict[str, ArmConfig] = field(default_factory=d1_arm_default_factory)
    # D1 自带夹爪就当作第 7 关节，不再单独建 endeffector
    endeffector: dict[str, EndEffectorConfig] = field(default_factory=lambda: {})

@RobotConfig.register_subclass("d1_only_arm")
@dataclass
class D1_OnlyArm_RobotConfig(UnitreeRobotConfig):
    # 不带摄像头
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {})
    # 只挂一只 D1 机械臂
    arm: dict[str, ArmConfig] = field(
        default_factory=lambda: {
            "d1": D1ArmConfig(
                motors=d1_motors,
                init_pose=np.zeros(7),
            )
        }
    )
    # 暂时没有独立 endeffector（夹爪当成第 7 关节）
    endeffector: dict[str, EndEffectorConfig] = field(default_factory=lambda: {})
