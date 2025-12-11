import json
import threading
import time
from typing import Optional

import numpy as np

try:
    # DDS 通讯接口
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    # D1 机械臂的 IDL 类型（名字可能因 SDK 版本略有不同，如有 import 报错，请根据本机 SDK 调整此处路径）
    from unitree_sdk2py.idl.unitree_arm.msg.dds_ import ArmString_, PubServoInfo_
except ImportError as e:  # pragma: no cover - 在未安装 SDK 的环境下不会被执行
    raise ImportError(
        "Import D1 的 Python SDK 失败。请确认已经安装 unitree_sdk2py 且包含 unitree_arm 的 IDL 绑定。"
    ) from e

from unitree_deploy.robot_devices.arm.configs import D1ArmConfig  # type: ignore
from unitree_deploy.robot_devices.robots_devices_utils import (
    DataBuffer,
    RobotDeviceAlreadyConnectedError,
    RobotDeviceNotConnectedError,
)
from unitree_deploy.utils.rich_logger import log_error, log_info, log_success, log_warning


class D1_ArmController:
    """D1 机械臂控制封装，接口对齐 Z1/G1，用于 unitree_deploy 的真实机器人控制。

    只实现最核心的一套接口：
      - connect / disconnect
      - motor_names
      - read_current_motor_q / read_current_arm_q / read_current_arm_dq
      - write_arm
      - arm_ik（暂不实现，返回 None）

    约定：
      - 与 Unifolm / unitree_deploy 上层一致，关节角使用「弧度制」表示。
      - D1 SDK 的关节角是「角度制（度）」，这里内部自动做 rad <-> deg 转换。
      - 第 7 维（索引 6）用作夹爪：[-1, 1] 线性映射到 [0, 65] mm。
    """

    def __init__(self, config: D1ArmConfig):
        self.config = config

        # 约定 motors: { name: (index, "d1-joint") }
        self.motors = config.motors
        self._motor_names = list(self.motors.keys())
        self._num_joints = len(self._motor_names)  # 预期 7

        # 缓存最新一帧关节角（弧度制）
        self.q_buffer = DataBuffer()

        # DDS 对象
        self.cmd_publisher: Optional[ChannelPublisher] = None
        self.servo_subscriber: Optional[ChannelSubscriber] = None

        self.subscribe_thread: Optional[threading.Thread] = None
        self._running = False

        self.is_connected = False
        self.unit_test = getattr(config, "unit_test", False)

        # 从说明书抄的关节角度范围（单位：度），先转成弧度以方便裁剪
        joint_deg_min = np.array([-135, -90, -90, -135, -90, -135], dtype=np.float32)
        joint_deg_max = np.array([+135, +90, +90, +135, +90, +135], dtype=np.float32)
        self.joint_rad_min = np.deg2rad(joint_deg_min)
        self.joint_rad_max = np.deg2rad(joint_deg_max)

        # 夹爪行程范围（mm）
        self.gripper_min_mm = 0.0
        self.gripper_max_mm = 65.0

    # ========================= 属性 & 必要接口 =========================

    @property
    def motor_names(self) -> list[str]:
        return self._motor_names

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    # -----------------------------------------------------------------

    def connect(self):
        """建立与 D1 的 DDS 通讯，订阅 current_servo_angle。"""
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "D1_ArmController is already connected. 不要重复调用 `connect()`。"
            )

        log_info("[D1_ArmController] 初始化 DDS 通讯...")
        # 初始化 DDS 工厂（与 G1/Z1 相同风格）
        ChannelFactoryInitialize(0)

        # 控制指令 publisher —— rt/arm_Command
        self.cmd_publisher = ChannelPublisher(self.config.topic_command, ArmString_)
        self.cmd_publisher.Init()

        # 关节角度 subscriber —— current_servo_angle
        self.servo_subscriber = ChannelSubscriber(self.config.topic_servo_angle, PubServoInfo_)
        self.servo_subscriber.Init(self._servo_callback)

        # 启动一个轻量级守护线程，保持订阅活跃（风格与 g1_arm 保持一致）
        self._running = True        self.subscribe_thread = threading.Thread(
            target=self._spin_subscriber_loop, name="d1._subscribe_motor_state"
        )
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # 等待第一帧关节角数据
        t0 = time.time()
        while self.q_buffer.get_data() is None and (time.time() - t0) < 5.0:
            time.sleep(0.01)
            log_warning("[D1_ArmController] Waiting to subscribe dds...")

        if self.q_buffer.get_data() is None:
            log_warning("[D1_ArmController] 5 秒内未收到关节数据，后续 read_current_arm_q() 将返回 0 向量。")

        self.is_connected = True
        log_success("[D1_ArmController] Connect OK!")

    def disconnect(self):
        """简单把运行标志置 false，退出守护线程。"""
        if not self.is_connected:
            return
        self._running = False
        self.is_connected = False
        log_info("[D1_ArmController] Disconnected.")

    # ========================= 状态读取 =========================

    def read_current_motor_q(self) -> np.ndarray:
        """与 read_current_arm_q 相同，返回 shape (7,) 的 np.ndarray（弧度制）。"""
        return self.read_current_arm_q()

    def read_current_arm_q(self) -> np.ndarray:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()
        data = self.q_buffer.get_data()
        if data is None:
            # 没有任何订阅数据时，返回零向量，避免上层崩溃
            return np.zeros(self._num_joints, dtype=np.float32)
        return np.asarray(data, dtype=np.float32).copy()

    def read_current_arm_dq(self) -> np.ndarray:
        """D1 的 DDS 接口没有提供速度，这里简单返回 0。"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()
        return np.zeros(self._num_joints, dtype=np.float32)

    # ========================= 写控制命令 =========================

    def write_arm(
        self,
        q_target: list[float] | np.ndarray,
        tauff_target: list[float] | np.ndarray | None = None,
        time_target: float | None = None,
        cmd_target: str | None = None,
    ):
        """发送多关节角控制命令（funcode=2）。

        参数
        ----
        q_target
            长度 7 的目标关节角：
              - 前 6 维：弧度制关节角
              - 第 7 维：[-1, 1] 的归一化夹爪开合（-1=全闭，1=全开）
        tauff_target / cmd_target
            D1 目前没有用到力矩控制，这里保留参数只是为了跟 G1/Z1 接口一致，内部会忽略。
        time_target
            上层会给一个“执行到达时间”，D1 当前协议只有 delay_ms，这里先写死 0（即时执行）。
        """
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()

        q_target = np.asarray(q_target, dtype=np.float32).reshape(-1)
        if q_target.shape[0] != self._num_joints:
            raise ValueError(f"[D1_ArmController] q_target 维度错误，期望 {self._num_joints}，实际 {q_target.shape[0]}")

        # ------------------- 关节角：弧度 -> 度，并做范围裁剪 -------------------
        joint_rad = q_target[:6]
        joint_rad = np.clip(joint_rad, self.joint_rad_min, self.joint_rad_max)
        joint_deg = np.rad2deg(joint_rad)

        # ------------------- 夹爪：[-1,1] -> [0, 65] mm -------------------
        gripper_norm = float(q_target[6])
        gripper_norm = float(np.clip(gripper_norm, -1.0, 1.0))
        gripper_mm = (
            (gripper_norm + 1.0) / 2.0 * (self.gripper_max_mm - self.gripper_min_mm) + self.gripper_min_mm
        )

        # ------------------- 构造 JSON 命令 -------------------
        payload = {
            "seq": int(time.time() * 1000) & 0xFFFFFFFF,  # 简单用时间戳当 seq
            "address": 1,
            "funcode": 2,
            "data": {
                "mode": 1,  # 使用轨迹平滑模式（官方文档：mode=1 轨迹使用的大平滑）
                "angle0": float(joint_deg[0]),
                "angle1": float(joint_deg[1]),
                "angle2": float(joint_deg[2]),
                "angle3": float(joint_deg[3]),
                "angle4": float(joint_deg[4]),
                "angle5": float(joint_deg[5]),
                "angle6": float(gripper_mm),
            },
        }

        if self.cmd_publisher is None:
            log_error("[D1_ArmController] cmd_publisher 还未初始化。")
            raise RobotDeviceNotConnectedError("cmd_publisher is None, did you call connect()?")

        msg = ArmString_()
        # 不同 SDK 版本字段名可能是 data 或 data_，根据你本地 IDL 调整这行：
        try:
            msg.data_ = json.dumps(payload, ensure_ascii=False)
        except AttributeError:
            msg.data = json.dumps(payload, ensure_ascii=False)

        self.cmd_publisher.Write(msg)

    # ========================= IK 接口占位 =========================

    def arm_ik(self, *args, **kwargs):
        """D1 暂不提供 IK 封装，这里返回 None，仅为兼容 Arm 协议。"""
        return None

    # ========================= 内部回调 & 线程 =========================

    def _servo_callback(self, msg: PubServoInfo_):
        """订阅 current_servo_angle 的回调，将角度从度转换成弧度并写入 buffer。"""
        try:
            # 字段名按 C++ 的 PubServoInfo_::servoX_data_ 推断，如果与你的 SDK 不一致，请据实修改
            joint_deg = np.array(
                [
                    msg.servo0_data_,
                    msg.servo1_data_,
                    msg.servo2_data_,
                    msg.servo3_data_,
                    msg.servo4_data_,
                    msg.servo5_data_,
                    msg.servo6_data_,
                ],
                dtype=np.float32,
            )
        except AttributeError:
            # 如果字段名不同，可以在这里打印 msg.__dict__ 做一次调试
            log_error(
                "[D1_ArmController] PubServoInfo_ 字段名与预期不符，请检查 D1 Python SDK 的 IDL 定义。"
            )
            return

        joint_rad = np.deg2rad(joint_deg[:6])
        gripper_mm = joint_deg[6]
        q = np.concatenate([joint_rad, np.array([gripper_mm], dtype=np.float32)], axis=0)
        self.q_buffer.set_data(q)

    def _spin_subscriber_loop(self):
        """保持 DDS 订阅线程存活。实际订阅工作由 SDK 内部线程完成，这里只做 keep-alive。"""
        while self._running:
            time.sleep(0.01)
