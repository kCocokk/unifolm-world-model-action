import json
import socket
import threading
import time
from typing import Optional

import numpy as np

from unitree_deploy.robot_devices.arm.configs import D1ArmConfig, MotorParams  # type: ignore
from unitree_deploy.robot_devices.robots_devices_utils import (
    RobotDeviceAlreadyConnectedError,
    RobotDeviceNotConnectedError,
)
from unitree_deploy.utils.joint_trajcetory_inter import JointTrajectoryInterpolator
from unitree_deploy.utils.rich_logger import log_error, log_info, log_success, log_warning


class D1_ArmController:
    """D1 机械臂控制封装（通过本地 TCP bridge，与 C++ D1 DDS 程序通信）。

    约定（与 d1_bridge.cpp 配套）：
      - bridge 程序监听 {bridge_host}:{bridge_port}（默认 127.0.0.1:5556）。
      - Python 通过简单文本协议与 bridge 通信：
          - 发送控制：  CMD <json>\n
          - 请求关节角：GET_Q\n
          - 返回关节角：Q <servo0_deg> ... <servo6_deg>\n

    关键设计（对齐 z1/g1）：
      - write_arm(...) 只更新目标（线程安全），后台控制线程以 control_dt 周期下发指令。
      - cmd_target 支持 "schedule_waypoint" / "drive_to_waypoint"。
      - 安全策略：不做“自动卸力”；发现异常则“停止运动=保持当前位置/最后安全目标”。
    """

    def __init__(self, config: Optional[D1ArmConfig] = None):
        # 允许用户直接 D1_ArmController() 做最小测试；
        # 正常在 robot_client 中会由 draccus/yaml 构造完整 config 并传入。
        if config is None:
            config = D1ArmConfig(motors={f"joint{i}": MotorParams(motor_id=i) for i in range(7)})
        self.config = config

        # motors: { name: (index, "d1-joint") }
        self.motors = config.motors
        self._motor_names = list(self.motors.keys())
        self._num_joints = len(self._motor_names)  # 预期 7（6 关节 + 1 夹爪）

        # 关节范围（单位：弧度）
        # D1 文档：J0 ±135°, J1 ±90°, J2 ±90°, J3 ±135°, J4 ±90°, J5 ±135°。
        # 第 7 维（夹爪/第 6 轴后级）各固件版本定义不一，这里给一个较宽的默认范围。
        joint_deg_min = np.array([-135, -90, -90, -135, -90, -135, -90], dtype=np.float32)
        joint_deg_max = np.array([+135, +90, +90, +135, +90, +135, +90], dtype=np.float32)
        safe_scale = float(getattr(config, "safe_range_scale", 1.0))
        safe_scale = float(np.clip(safe_scale, 0.1, 1.0))
        center = (joint_deg_min + joint_deg_max) / 2.0
        half = (joint_deg_max - joint_deg_min) / 2.0 * safe_scale
        self.joint_rad_min = np.deg2rad(center - half)
        self.joint_rad_max = np.deg2rad(center + half)

        # TCP 连接（默认 127.0.0.1:5556，与你的 d1_bridge_slave 一致）
        self.bridge_host = getattr(config, "bridge_host", "127.0.0.1")
        self.bridge_port = int(getattr(config, "bridge_port", 5556))

        self.sock: Optional[socket.socket] = None
        self.f: Optional[object] = None  # file-like，用于 readline / write
        self.is_connected = False

        # 通讯锁：避免 CMD 与 GET_Q 互相打乱
        self.io_lock = threading.RLock()

        # 控制锁：保护目标与状态
        self.ctrl_lock = threading.RLock()

        # 简单 seq 计数器（仅用于 JSON 里）
        self._seq_counter = int(time.time() * 1000) & 0xFFFFFFFF

        # 控制线程
        self.control_dt = float(getattr(config, "control_dt", 0.1))
        self.max_pos_speed = float(getattr(config, "max_pos_speed", 180 * (np.pi / 180) * 2))
        # 第 7 维也按“关节角”处理：弧度/秒。
        self.max_gripper_speed = float(getattr(config, "max_gripper_speed", self.max_pos_speed))

        # 只在 connect() 成功后置 True，避免外部逻辑多次触发导致异常。
        self.auto_power_enable_flag = False
        self._stop_event = threading.Event()
        self._ctrl_thread: Optional[threading.Thread] = None
        self._fb_thread: Optional[threading.Thread] = None

        # 插值器与缓存
        self.pose_interp: Optional[JointTrajectoryInterpolator] = None
        self._last_waypoint_time: Optional[float] = None
        self._last_cmd_time_monotonic: float = 0.0

        # 目标：内部统一为 **弧度**（7 维全是关节角 rad）
        init_pose = np.array(config.init_pose, dtype=np.float32) if config.init_pose is not None else np.zeros(self._num_joints, dtype=np.float32)
        if init_pose.shape[0] != self._num_joints:
            init_pose = np.zeros(self._num_joints, dtype=np.float32)

        self._arm_q_target = init_pose.copy()
        self._arm_tauff_target = np.zeros(self._num_joints, dtype=np.float32)
        self._arm_time_target: Optional[float] = None
        self._arm_cmd_target: Optional[str] = None

        # “停止运动”标志：保持当前位置/最后安全姿态
        self._stop_motion = False
        self._stop_pose = init_pose.copy()

        # 兼容：某些脚本/日志会访问该 flag
        self.auto_power_enable_flag: bool = False

        # 反馈缓存（尽量提供给 capture_observation 使用）
        self._last_q_measured = init_pose.copy()
        self._last_q_measured_ts = 0.0

    # ========================= 一些属性 =========================

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
        # buffering=0, 不需要 flush

    def _read_line(self, timeout: float = 1.0) -> str:
        if self.sock is None or self.f is None:
            raise RobotDeviceNotConnectedError("socket/file is None, did you call connect()?")

        self.sock.settimeout(timeout)
        line = self.f.readline()
        if not line:
            raise TimeoutError("bridge readline timeout/EOF")
        try:
            return line.decode("utf-8", errors="ignore").strip()
        except Exception:
            return str(line)

    def _send_json_command(self, funcode: int, data: dict):
        """发送 D1 JSON 指令（不等待 ACK）。"""
        payload = {
            "seq": int(self._next_seq()),
            "address": 1,
            "funcode": int(funcode),
            "data": data,
        }
        json_str = json.dumps(payload, ensure_ascii=False)
        cmd_line = "CMD " + json_str
        self._send_line(cmd_line)

    def _clip_q(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float32).copy()
        if q.shape[0] != self._num_joints:
            raise ValueError(f"q dim error, expect {self._num_joints}, got {q.shape[0]}")
        q = np.clip(q, self.joint_rad_min, self.joint_rad_max)
        return q

    # ========================= 电机供电 / 使能 / 归零 =========================

    def set_power(self, on: bool):
        """funcode=6：power=0 断电，power=1 上电"""
        power_val = 1 if on else 0
        with self.io_lock:
            self._send_json_command(funcode=6, data={"power": power_val})

    def set_all_damping_raw(self, mode: int):
        """funcode=5：mode 0~80000，0 卸力，80000 完全锁死（注意：这里的“锁死”是高阻尼/高保持）。"""
        mode = int(np.clip(mode, 0, 80000))
        with self.io_lock:
            self._send_json_command(funcode=5, data={"mode": mode})

    def set_all_damping(self, stiffness: float):
        """stiffness ∈ [0,1] 映射到 mode ∈ [0,80000]"""
        stiffness = float(np.clip(stiffness, 0.0, 1.0))
        self.set_all_damping_raw(int(stiffness * 80000))

    def enable_all(self):
        """完全使能（高保持）。

        D1 的固件/SDK版本可能对“使能”与“阻尼(保持)”解释不同；
        为了尽可能把机械臂从“卸力”状态拉起来，这里同时：
          1) 对每个关节发送 funcode=4 enable=1
          2) 发送 funcode=5 mode=80000（高保持）
        """
        # per-joint enable
        for jid in range(7):
            try:
                self.set_joint_enable(jid, True)
                time.sleep(0.02)
            except Exception:
                pass

        # high stiffness hold
        self.set_all_damping_raw(80000)

    def disable_all(self):
        """完全卸力（⚠️你不希望自动卸力，这个函数保留给手动调试，不会自动调用）"""
        self.set_all_damping_raw(0)

    def set_joint_enable(self, joint_id: int, enable: bool, raw_mode: Optional[int] = None):
        """funcode=4：单个关节使能/卸力"""
        if not (0 <= joint_id <= 6):
            raise ValueError(f"joint_id out of range: {joint_id}")
        if raw_mode is None:
            mode = 1 if enable else 0
        else:
            mode = int(raw_mode)
        with self.io_lock:
            self._send_json_command(funcode=4, data={"id": int(joint_id), "mode": mode})

    def go_zero_pose(self):
        """funcode=7：位姿归零"""
        with self.io_lock:
            self._send_json_command(funcode=7, data={})

    # ========================= 连接 / 断开 =========================

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError("D1_ArmController already connected.")

        log_info(f"[D1_ArmController] Connecting to bridge {self.bridge_host}:{self.bridge_port} ...")
        self.sock = socket.create_connection((self.bridge_host, self.bridge_port), timeout=5.0)
        self.f = self.sock.makefile("rwb", buffering=0)
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

        # 初始化插值器与目标为当前姿态（优先真实值，否则退化为 init_pose）
        try:
            q = self._read_q_from_bridge(timeout=2.0)
            with self.ctrl_lock:
                self._last_q_measured = q.copy()
                self._last_q_measured_ts = time.monotonic()
                self._arm_q_target = q.copy()
                self._stop_pose = q.copy()
            log_info(f"[D1_ArmController] First q from bridge (rad+norm): {q}")
        except Exception as e:
            log_warning(f"[D1_ArmController] 首次读取关节角失败，使用 init_pose: {e}")

        self.pose_interp = JointTrajectoryInterpolator(times=[time.monotonic()], joint_positions=[self._arm_q_target.copy()])
        self._last_waypoint_time = self.pose_interp.times[-1]

        # 启动线程
        self._stop_event.clear()
        self._ctrl_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self._ctrl_thread.start()

        self._fb_thread = threading.Thread(target=self._feedback_poll_loop, daemon=True)
        self._fb_thread.start()

        log_success("[D1_ArmController] Connect OK!")

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("D1_ArmController not connected.")

        # 先停线程（不自动卸力）
        self._stop_event.set()
        try:
            if self._ctrl_thread is not None:
                self._ctrl_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            if self._fb_thread is not None:
                self._fb_thread.join(timeout=2.0)
        except Exception:
            pass

        try:
            with self.io_lock:
                if self.f is not None:
                    self.f.close()
                if self.sock is not None:
                    self.sock.close()
        except Exception:
            pass

        self.f = None
        self.sock = None
        self.is_connected = False
        log_info("[D1_ArmController] Disconnected.")

    # ========================= 状态读取 =========================

    def _read_q_from_bridge(self, timeout: float = 1.0) -> np.ndarray:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()
        with self.io_lock:
            self._send_line("GET_Q")
            line = self._read_line(timeout=timeout)

        if not line.startswith("Q "):
            raise RuntimeError(f"Unexpected GET_Q reply: {line}")

        parts = line.split()
        if len(parts) != 1 + self._num_joints:
            raise RuntimeError(f"GET_Q reply length mismatch: {line}")

        # d1_bridge_slave 直接把 PubServoInfo 的 servo*_data_ 作为角度(°)透传回来。
        # 这里统一按 **degree->radian** 做转换，不再把最后一维当作 mm 或归一化值。
        vals = np.asarray([float(x) for x in parts[1:]], dtype=np.float32)
        q_rad = np.deg2rad(vals)
        return self._clip_q(q_rad)

    def read_current_motor_q(self) -> np.ndarray:
        return self.read_current_arm_q()

    def read_current_arm_q(self) -> np.ndarray:
        """尽量返回缓存的真实反馈（rad）。若缓存尚未更新，则尝试现场 GET_Q。"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()

        with self.ctrl_lock:
            q = self._last_q_measured.copy()
            ts = self._last_q_measured_ts

        # 缓存新鲜则直接返回
        if ts > 0 and (time.monotonic() - ts) < max(0.5, 5 * self.control_dt):
            return q

        # 否则尝试主动拉一次
        try:
            q2 = self._read_q_from_bridge(timeout=1.0)
            with self.ctrl_lock:
                self._last_q_measured = q2.copy()
                self._last_q_measured_ts = time.monotonic()
            return q2
        except Exception as e:
            log_warning(f"[D1_ArmController] read_current_arm_q fallback to cached, err={e}")
            return q

    def read_current_arm_dq(self) -> np.ndarray:
        """bridge 没有速度信息，这里简单返回 0。"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()
        return np.zeros(self._num_joints, dtype=np.float32)

    # ========================= 控制接口（对齐 z1/g1） =========================

    def stop_motion(self, reason: str = "manual"):
        """停止运动：保持当前位置（或最后安全姿态），不卸力。"""
        with self.ctrl_lock:
            self._stop_motion = True
            try:
                hold = self._read_q_from_bridge(timeout=0.5)
            except Exception:
                hold = self._last_q_measured.copy()
            self._stop_pose = hold.copy()
            self._arm_q_target = hold.copy()
            self._arm_cmd_target = "drive_to_waypoint"
            self._arm_time_target = None
        log_warning(f"[D1_ArmController] stop_motion: {reason}")

    def clear_stop(self):
        with self.ctrl_lock:
            self._stop_motion = False

    def write_arm(
        self,
        q_target: list[float] | np.ndarray,
        tauff_target: list[float] | np.ndarray = None,
        time_target: float | None = None,
        cmd_target: str | None = None,
    ):
        """设置控制目标（线程安全，后台线程按 control_dt 下发）。"""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError()

        q_target = np.asarray(q_target, dtype=np.float32).reshape(-1)
        if q_target.shape[0] != self._num_joints:
            raise ValueError(f"[D1_ArmController] q_target dim error, expect {self._num_joints}, got {q_target.shape[0]}")
        q_target = self._clip_q(q_target)

        if tauff_target is None:
            tauff = np.zeros_like(q_target, dtype=np.float32)
        else:
            tauff = np.asarray(tauff_target, dtype=np.float32).reshape(-1)
            if tauff.shape[0] != self._num_joints:
                tauff = np.zeros_like(q_target, dtype=np.float32)

        with self.ctrl_lock:
            # 新指令到来时，解除 stop（由外部明确 stop_motion 才会进入 stop 状态）
            self._stop_motion = False

            self._arm_q_target = q_target
            self._arm_tauff_target = tauff
            self._arm_time_target = time_target
            self._arm_cmd_target = cmd_target
            self._last_cmd_time_monotonic = time.monotonic()

    # ========================= 内部：下发与线程循环 =========================

    def _send_q_cmd(self, q: np.ndarray):
        """将 q（rad）转换为 D1 角度（deg）并下发。"""
        q = self._clip_q(q)
        joint_deg = np.rad2deg(q).astype(np.float32)

        data = {
            "mode": 1,  # 轨迹大平滑
            "angle0": float(joint_deg[0]),
            "angle1": float(joint_deg[1]),
            "angle2": float(joint_deg[2]),
            "angle3": float(joint_deg[3]),
            "angle4": float(joint_deg[4]),
            "angle5": float(joint_deg[5]),
            "angle6": float(joint_deg[6]),
        }

        with self.io_lock:
            self._send_json_command(funcode=2, data=data)

    def _drive_to_waypoint(self, target_pose: np.ndarray, t_insert_time: float = 0.8):
        """类似 g1/z1：在 t_now + t_insert_time 插入一个目标点。"""
        assert self.pose_interp is not None
        t_now = time.monotonic()
        curr_time = t_now + self.control_dt
        target_time = t_now + max(self.control_dt, float(t_insert_time))

        self.pose_interp = self.pose_interp.drive_to_waypoint(
            pose=target_pose,
            time=target_time,
            curr_time=curr_time,
            max_pos_speed=self.max_pos_speed,
        )
        self._last_waypoint_time = self.pose_interp.times[-1]

    def _schedule_waypoint(
        self,
        arm_q_target: np.ndarray,
        arm_time_target: float | None,
        t_now: float,
        start_time: float,
        last_waypoint_time: float | None,
    ) -> float | None:
        """对齐 g1/z1 的 schedule 语义（arm_time_target 来自 perf_counter 时钟）。"""
        assert self.pose_interp is not None

        curr_time = t_now + self.control_dt
        if arm_time_target is None:
            # 没有给目标时间：默认在“下一拍”插入
            target_time = curr_time + self.control_dt
        else:
            # 将 perf_counter 时间换算到 monotonic 基准
            target_time = time.monotonic() - time.perf_counter() + float(arm_time_target)
            # 目标时间不能早于下一拍
            target_time = max(target_time, curr_time + self.control_dt)

        self.pose_interp = self.pose_interp.schedule_waypoint(
            pose=arm_q_target,
            time=target_time,
            max_pos_speed=self.max_pos_speed,
            curr_time=curr_time,
            last_waypoint_time=last_waypoint_time,
        )
        return self.pose_interp.times[-1]

    def _ctrl_motor_state(self):
        """后台控制线程：按 control_dt 执行插值并下发 CMD。"""
        assert self.pose_interp is not None
        last_waypoint_time = self._last_waypoint_time
        start_time = time.perf_counter()

        while not self._stop_event.is_set():
            loop_t0 = time.perf_counter()
            try:
                t_now = time.monotonic()

                with self.ctrl_lock:
                    stop_motion = self._stop_motion
                    stop_pose = self._stop_pose.copy()
                    arm_q_target = self._arm_q_target.copy()
                    arm_time_target = self._arm_time_target
                    arm_cmd = self._arm_cmd_target

                if stop_motion:
                    # 保持停止姿态：把插值器重置为单点
                    self.pose_interp = JointTrajectoryInterpolator(times=[t_now], joint_positions=[stop_pose])
                    last_waypoint_time = self.pose_interp.times[-1]
                else:
                    # 执行上层命令（对齐 g1/z1）
                    if arm_cmd == "drive_to_waypoint":
                        self._drive_to_waypoint(target_pose=arm_q_target, t_insert_time=0.8)
                        last_waypoint_time = self.pose_interp.times[-1]
                    elif arm_cmd == "schedule_waypoint":
                        last_waypoint_time = self._schedule_waypoint(
                            arm_q_target=arm_q_target,
                            arm_time_target=arm_time_target,
                            t_now=t_now,
                            start_time=start_time,
                            last_waypoint_time=last_waypoint_time,
                        )

                # 计算当前要下发的 q
                q_cmd = self.pose_interp(t_now)
                q_cmd = self._clip_q(q_cmd)

                # 发送
                self._send_q_cmd(q_cmd)

                # 更新“最后安全姿态”（供 stop_motion/异常时使用）
                with self.ctrl_lock:
                    self._stop_pose = q_cmd.copy()

            except Exception as e:
                log_error(f"[D1_ArmController] ctrl loop exception: {e}")
                # 异常时：停止运动（保持最后安全姿态），不卸力
                try:
                    self.stop_motion(reason=f"ctrl_exception: {e}")
                except Exception:
                    pass

            # sleep 到下个控制周期
            elapsed = time.perf_counter() - loop_t0
            time.sleep(max(0.0, self.control_dt - elapsed))

    def _feedback_poll_loop(self):
        """后台反馈轮询：周期 GET_Q，更新缓存。"""
        poll_dt = float(getattr(self.config, "feedback_dt", max(self.control_dt, 0.1)))
        while not self._stop_event.is_set():
            t0 = time.perf_counter()
            try:
                q = self._read_q_from_bridge(timeout=1.0)
                with self.ctrl_lock:
                    self._last_q_measured = q.copy()
                    self._last_q_measured_ts = time.monotonic()
            except Exception:
                # 反馈失败不影响控制线程
                pass
            time.sleep(max(0.0, poll_dt - (time.perf_counter() - t0)))

    # ========================= IK 占位 =========================

    def arm_ik(self, *args, **kwargs):
        return None