import abc
from dataclasses import dataclass

import draccus
import numpy as np


@dataclass
class ArmConfig(draccus.ChoiceRegistry, abc.ABC):
    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)


@ArmConfig.register_subclass("z1")
@dataclass
class Z1ArmConfig(ArmConfig):
    motors: dict[str, tuple[int, str]]

    init_pose: list = None
    unit_test: bool = False
    control_dt: float = 1 / 500.0

    robot_kp: np.ndarray = np.array([4, 6, 6, 6, 6, 6])
    robot_kd: np.ndarray = np.array([350, 300, 300, 200, 200, 200])
    max_pos_speed: float = 180 * (np.pi / 180) * 2
    log_level: str | int = "ERROR"

    def __post_init__(self):
        if self.control_dt < 0.002:
            raise ValueError(f"`control_dt` must > 1/500 (got {self.control_dt})")


@ArmConfig.register_subclass("z1_dual")
@dataclass
class Z1DualArmConfig(ArmConfig):
    left_robot_ip: str
    left_robot_port1: int
    left_robot_port2: int
    right_robot_ip: str
    right_robot_port1: int
    right_robot_port2: int
    motors: dict[str, tuple[int, str]]

    robot_kp: np.ndarray = np.array([4, 6, 6, 6, 6, 6])
    robot_kd: np.ndarray = np.array([350, 300, 300, 200, 200, 200])
    mock: bool = False
    unit_test: bool = False
    init_pose_left: list | None = None
    init_pose_right: list | None = None
    max_pos_speed: float = 180 * (np.pi / 180) * 2
    control_dt: float = 1 / 500.0

    def __post_init__(self):
        if self.control_dt < 0.002:
            raise ValueError(f"`control_dt` must > 1/500 (got {self.control_dt})")


@ArmConfig.register_subclass("g1")
@dataclass
class G1ArmConfig(ArmConfig):
    motors: dict[str, tuple[int, str]]
    mock: bool = False
    unit_test: bool = False
    init_pose: np.ndarray | list = np.zeros(14)

    control_dt: float = 1 / 500.0
    max_pos_speed: float = 180 * (np.pi / 180) * 2

    topic_low_command: str = "rt/lowcmd"
    topic_low_state: str = "rt/lowstate"

    kp_high: float = 300.0
    kd_high: float = 3.0
    kp_low: float = 80.0  # 140.0
    kd_low: float = 3.0  # 3.0
    kp_wrist: float = 40.0
    kd_wrist: float = 1.5

    def __post_init__(self):
        if self.control_dt < 0.002:
            raise ValueError(f"`control_dt` must > 1/500 (got {self.control_dt})")


@ArmConfig.register_subclass("d1")
@dataclass
class D1ArmConfig(ArmConfig):
    """D1 机械臂的配置，模仿 Z1/G1 的风格。"""

    motors: dict[str, tuple[int, str]]

    # 初始姿态（弧度制），长度 7（6 关节 + 1 夹爪）
    init_pose: list | None = None
    unit_test: bool = False

    # D1 的控制周期为 10Hz
    control_dt: float = 0.1

    # DDS / 网络相关配置
    ip: str = "192.168.123.100"
    topic_command: str = "rt/arm_Command"
    topic_feedback: str = "rt/arm_Feedback"
    topic_servo_angle: str = "current_servo_angle"

    log_level: str | int = "ERROR"

    def __post_init__(self):
        # 这里不做很严格的 dt 限制，只要 >0 即可
        if self.control_dt <= 0:
            raise ValueError(f"`control_dt` must > 0 (got {self.control_dt})")