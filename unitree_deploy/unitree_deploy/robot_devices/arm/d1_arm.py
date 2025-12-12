import json
import socket
import time
from typing import Optional

import numpy as np

from unitree_deploy.robot_devices.arm.configs import D1ArmConfig  # type: ignore
from unitree_deploy.robot_devices.robots_devices_utils import (
    RobotDeviceAlreadyConnectedError,
    RobotDeviceNotConnectedError,
)
from unitree_deploy.utils.rich_logger import log_error, log_info, log_success, log_warning


class D1_ArmController:
    """D1 机械臂控制封装（通过本地 TCP bridge，与 C++ D1 DDS 程序通信）。

    约定：
      - bridge 程序监听 127.0.0.1:5555（见 d1_bridge.cpp）。
      - Python 通过简单文本协议与 bridge 通信：
          - 发送控制：  CMD <json>\n
          - 请求关节角：GET_Q\n
          - 返回关节角：Q <servo0_deg> ... <servo6_deg>\n

    对上层暴露接口：
      - connect / disconnect
      - motor_names / motor_models / motor_indices
      - read_current_motor_q / read_current_arm_q / read_current_arm_dq
      - write_arm
      - set_power / enable_all / disable_all / go_zero_pose 等（内部基于 funcode=2/4/5/6/7 构造 JSON）
    """

    def __init__(self, config: D1ArmConfig):
        self.config = config

        # motors: { name: (index, "d1-joint") }
        self.motors = config.motors
        self._motor_names = list(self.motors.keys())
        self._num_joints = len(self._motor_names)  # 预期 7

        # 关节范围（前 6 关节，单位：弧度）
        joint_deg_min = np.array([-135, -90, -90, -135, -90, -135], dtype=np.float32)
        joint_deg_max = np.array([+135, +90, +90, +135, +90, +135], dtype=np.float32)
        self.joint_rad_min = np.deg2rad(joint_deg_min)
        self.joint_rad_max = np.deg2rad(joint_deg_max)

        # 夹爪范围（mm）
        self.gripper_min_mm = 0.0
        self.gripper_max_mm = 65.0

        # TCP 连接（默认 127.0.0.1:5555，可在 D1ArmConfig 里扩展 bridge_host/bridge_port 覆盖）
        self.bridge_host = getattr(config, "bridge_host", "127.0.0.1")
        self.bridge_port = int(getattr(config, "bridge_port", 5555))

        self.sock: Optional[socket.socket] = None
        self.f: Optional[object] = None  # file-like，用于 readline / write

        self.is_connected = False

        # 简单 seq 计数器（仅用于 JSON 里）
        self._seq_counter = int(time.time() * 1000) & 0xFFFFFFFF

    # ========================= 属性 =========================

    @property
    def motor_names(self) -> list[str]:
        return self._motor_names

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    # ========================= 内部工具 =========================

    def _next_seq(self) -> int:
        self._seq_counter = (self._seq_counter + 1) & 0xFFFFFFFF
        return self._seq_counter

    def _send_line(self, line: str):
        if self.f is None:
            raise RobotDeviceNotConnectedError("bridge file is None, did you call connect()?")

        data = (line + "\n").encode("utf-8")
        self.f.write(data)
        self.f.flush()

    def _read_line(self, timeout: float = 2.0) -> str:
        if self.f is None:
            raise RobotDeviceNotConnectedError("bridge file is None, did you call connect()?")

        # 简单超时处理：把 socket 设置成超时
        if self.sock is not None:
            self.sock.settimeout(timeout)
        line = self.f.readline()
        if not line:
            raise RobotDeviceNotConnectedError("bridge closed connection")
        return line.decode("utf-8", errors="replace").strip()

    def _send_json_command(self, funcode: int, data: dict, address: int = 1, seq: Optional[int] = None):
        if seq is None:
            seq = self._next_seq()
        payload = {
            "seq": int(seq),
            "address": int(address),
            "funcode": int(funcode),
            "data": data,
        }
        json_str = json.dumps(payload, ensure_ascii=False)
        cmd_line = "CMD " + json_str
        self._send_line(cmd_line)
        log_info(f"[D1_ArmController] Send cmd funcode={funcode}, data={data}")

    # ========================= 电机供电 / 使能 / 归零 =========================

    def set_power(self, on: bool):
        """funcode=6：power=0 断电，power=1 上电"""
        power_val = 1 if on else 0
        self._send_json_command(
            funcode=6,
            data={"power": power_val},
        )

    def set_all_damping_raw(self, mode: int):
        """funcode=5：mode 0~80000，0 卸力，80000 完全锁死"""
        mode = int(np.clip(mode, 0, 80000))
        self._send_json_command(
            funcode=5,
            data={"mode": mode},
        )

    def set_all_damping(self, stiffness: float):
        """stiffness ∈ [0,1] 映射到 mode ∈ [0,80000]"""
        stiffness = float(np.clip(stiffness, 0.0, 1.0))
        mode = int(stiffness * 80000)
        self.set_all_damping_raw(mode)

    def enable_all(self):
        """完全使能"""
        self.set_all_damping_raw(80000)

    def disable_all(self):
        """完全卸力"""
        self.set_all_damping_raw(0)

    def set_joint_enable(self, joint_id: int, enable: bool, raw_mode: Optional[int] = None):
        """funcode=4：单个关节使能/卸力"""
        if not (0 <= joint_id <= 6):
            raise ValueError(f"joint_id out of range: {joint_id}")
        if raw_mode is not None:
            mode = int(np.clip(raw_mode, 0, 80000))
        else:
            mode = 80000 if enable else 0
        self._send_json_command(
            funcode=4,
            data={"id": int(joint_id), "mode": mode},
        )

    def go_zero_pose(self):
        """funcode=7：位姿归零"""
        self._send_json_command(funcode=7, data={})

    # ========================= 连接 / 断开 =========================

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError("D1_ArmController already connected.")

        log_info(f"[D1_ArmController] Connecting to bridge {self.bridge_host}:{self.bridge_port} ...")
        self.sock = socket.create_connection((self.bridge_host, self.bridge_port), timeout=5.0)
        self.f = self.sock.makefile("rwb", buffering=0)

        # 标记为已连接（这样后面的 read_current_arm_q 就不会再抛 NotConnectedError）
        self.is_connected = True

        # 尝试上电 + 使能（允许失败）
        try:
            log_info("[D1_ArmController] Power ON")
            self.set_power(True)
            time.sleep(0.2)
            log_info("[D1_ArmController] Enable all joints")
            self.enable_all()
        except Exception as e:
            log_warning(f"[D1_ArmController] 上电/使能失败（可稍后手动重试）: {e}")

        # 试着读一次关节角，只是为了打印日志
        try:
            q = self.read_current_arm_q()
            log_info(f"[D1_ArmController] First q (rad) from bridge: {q}")
        except Exception as e:
            log_warning(f"[D1_ArmController] 首次读取关节角失败: {e}")

        log_success("[D1_ArmController] Connect OK!")


    def disconnect(self):
        if not self.is_connected:
            return

        log_info("[D1_ArmController] Disconnect: disable_all + power_off + close socket ...")
        try:
            self.disable_all()
            time.sleep(0.1)
            self.set_power(False)
        except Exception as e:
            log_warning(f"[D1_ArmController] 断开前卸力/断电失败: {e}")

        try:
            # 通知 bridge 可以关闭当前 client（可选）
            self._send_line("EXIT")
        except Exception:
            pass

        try:
            if self.f is not None:
                self.f.close()
        except Exception:
            pass

        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass

        self.f = None
        self.sock = None
        self.is_connected = False
        log_info("[D1_ArmController] Disconnected.")

    # ========================= 状态读取 =========================

    def read_current_motor_q(self) -> np.ndarray:
        return self.read_current_arm_q()

    def read_current_arm_q(self) -> np.ndarray:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()
        # 向 bridge 请求一帧当前关节角（度）
        self._send_line("GET_Q")
        line = self._read_line(timeout=2.0)
        if not line.startswith("Q "):
            log_warning(f"[D1_ArmController] Unexpected GET_Q reply: {line}")
            return np.zeros(self._num_joints, dtype=np.float32)

        parts = line.split()
        if len(parts) != 1 + self._num_joints:
            log_warning(f"[D1_ArmController] GET_Q reply length mismatch: {line}")
            return np.zeros(self._num_joints, dtype=np.float32)

        # 解析度数（前 6 个是关节角度，最后一个是夹爪行程 mm）
        try:
            vals = [float(x) for x in parts[1:]]
        except ValueError:
            log_warning(f"[D1_ArmController] GET_Q reply parse error: {line}")
            return np.zeros(self._num_joints, dtype=np.float32)

        joint_deg = np.asarray(vals[:6], dtype=np.float32)
        gripper_mm = float(vals[6])

        joint_rad = np.deg2rad(joint_deg)
        q = np.concatenate([joint_rad, np.array([gripper_mm], dtype=np.float32)], axis=0)
        return q

    def read_current_arm_dq(self) -> np.ndarray:
        """bridge 没有速度信息，这里简单返回 0。"""
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
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()

        q_target = np.asarray(q_target, dtype=np.float32).reshape(-1)
        if q_target.shape[0] != self._num_joints:
            raise ValueError(f"[D1_ArmController] q_target dim error, expect {self._num_joints}, got {q_target.shape[0]}")

        # 关节：弧度 -> 度 + 范围裁剪
        joint_rad = q_target[:6]
        joint_rad = np.clip(joint_rad, self.joint_rad_min, self.joint_rad_max)
        joint_deg = np.rad2deg(joint_rad)

        # 夹爪：[-1,1] -> [0,65] mm
        gripper_norm = float(q_target[6])
        gripper_norm = float(np.clip(gripper_norm, -1.0, 1.0))
        gripper_mm = (
            (gripper_norm + 1.0) / 2.0 * (self.gripper_max_mm - self.gripper_min_mm) + self.gripper_min_mm
        )

        data = {
            "mode": 1,  # 轨迹大平滑
            "angle0": float(joint_deg[0]),
            "angle1": float(joint_deg[1]),
            "angle2": float(joint_deg[2]),
            "angle3": float(joint_deg[3]),
            "angle4": float(joint_deg[4]),
            "angle5": float(joint_deg[5]),
            "angle6": float(gripper_mm),
        }

        self._send_json_command(
            funcode=2,
            data=data,
        )

    # ========================= IK 占位 =========================

    def arm_ik(self, *args, **kwargs):
        return None
