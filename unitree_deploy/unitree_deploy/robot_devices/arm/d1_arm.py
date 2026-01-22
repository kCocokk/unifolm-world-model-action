import json
import socket
import threading
import time
from typing import Optional, Union

import numpy as np

from unitree_deploy.robot_devices.arm.configs import D1ArmConfig  # type: ignore
from unitree_deploy.robot_devices.robots_devices_utils import (
    RobotDeviceAlreadyConnectedError,
    RobotDeviceNotConnectedError,
)
from unitree_deploy.utils.joint_trajcetory_inter import JointTrajectoryInterpolator
from unitree_deploy.utils.rich_logger import log_error, log_info, log_success, log_warning


class D1_ArmController:
    """D1 机械臂控制（通过本地 TCP bridge，与 C++ d1_bridge.cpp / d1_bridge_slave.cpp 通信）。

    约定（与 bridge 配套）：
      - bridge 监听 {bridge_host}:{bridge_port}
      - Python <-> bridge 文本协议：
          - 发送控制：  CMD <json>\n
          - 请求关节角：GET_Q\n
          - 返回关节角：Q <deg0> <deg1> ... <deg6>\n

    对齐 unitree_deploy 的统一接口（z1/g1）：
      - connect()/disconnect()
      - read_current_arm_q(): 返回 7D (rad)，包含 6 关节 + 夹爪（夹爪用“等效角度”的 rad 表示）
      - write_arm(q_target, time_target, cmd_target): 只更新目标；后台线程按 config.control_dt(默认 0.1s) 下发
    """

    def __init__(self, config: Optional[D1ArmConfig] = None):
        if config is None:
            # Standalone fallback
            config = D1ArmConfig(
                motors={
                    "J0": (0, "d1-joint"),
                    "J1": (1, "d1-joint"),
                    "J2": (2, "d1-joint"),
                    "J3": (3, "d1-joint"),
                    "J4": (4, "d1-joint"),
                    "J5": (5, "d1-joint"),
                    "J6": (6, "d1-joint"),
                }
            )

        self.config = config

        # ----- motor meta -----
        self.motors = config.motors
        self._motor_names = list(self.motors.keys())
        self._num_joints = len(self._motor_names)
        if self._num_joints != 7:
            log_warning(f"[D1_ArmController] expect 7 joints, got {self._num_joints}. Proceed anyway.")

        # ----- joint limits (deg) -----
        # D1 doc: J0 ±135, J1 ±90, J2 ±90, J3 ±135, J4 ±90, J5 ±135
        joint_deg_min = np.array([-135, -90, -90, -135, -90, -135], dtype=np.float32)
        joint_deg_max = np.array([+135, +90, +90, +135, +90, +135], dtype=np.float32)

        # 夹爪：用户给出的实测范围（deg）
        #   -18.6: 夹爪在中间（接近闭合）
        #   76.5 : 夹爪在两边（张开）
        # 允许通过环境变量覆盖（方便不同固件/标定）：
        #   export D1_GRIPPER_DEG_MIN=-18.6
        #   export D1_GRIPPER_DEG_MAX=76.5
        gmin = float(getattr(config, "gripper_deg_min", float(np.nan)))
        gmax = float(getattr(config, "gripper_deg_max", float(np.nan)))
        if not np.isfinite(gmin):
            try:
                import os

                gmin = float(os.environ.get("D1_GRIPPER_DEG_MIN", "-18.6"))
            except Exception:
                gmin = -18.6
        if not np.isfinite(gmax):
            try:
                import os

                gmax = float(os.environ.get("D1_GRIPPER_DEG_MAX", "76.5"))
            except Exception:
                gmax = 76.5

        gripper_deg_min = np.array([gmin], dtype=np.float32)
        gripper_deg_max = np.array([gmax], dtype=np.float32)

        # 安全缩放：只对前 6 关节做缩放，夹爪不缩放（范围本来就不大）
        safe_scale = float(getattr(config, "safe_range_scale", 1.0))
        safe_scale = float(np.clip(safe_scale, 0.1, 1.0))
        center = (joint_deg_min + joint_deg_max) / 2.0
        half = (joint_deg_max - joint_deg_min) / 2.0 * safe_scale
        joint_deg_min_s = center - half
        joint_deg_max_s = center + half

        self.joint_deg_min = np.concatenate([joint_deg_min_s, gripper_deg_min], axis=0)
        self.joint_deg_max = np.concatenate([joint_deg_max_s, gripper_deg_max], axis=0)
        self.joint_rad_min = np.deg2rad(self.joint_deg_min)
        self.joint_rad_max = np.deg2rad(self.joint_deg_max)

        # ----- bridge tcp -----
        self.bridge_host = getattr(config, "bridge_host", "127.0.0.1")
        self.bridge_port = int(getattr(config, "bridge_port", 5555))
        self.sock: Optional[socket.socket] = None
        self.f = None  # file-like, for readline/write
        self.is_connected = False

        # ----- locks -----
        self.io_lock = threading.RLock()  # avoid GET_Q/CMD interleave
        self.ctrl_lock = threading.RLock()

        # ----- control params -----
        self.control_dt = float(getattr(config, "control_dt", 0.1))  # D1 firmware control ~10Hz
        self.feedback_dt = float(getattr(config, "feedback_dt", max(self.control_dt, 0.1)))
        self.max_pos_speed = float(getattr(config, "max_pos_speed", 120 * (np.pi / 180)))  # rad/s

        # ----- motion comfort / safety (recommended) -----
        # Speed limits are enforced *in addition* to interpolator speed limits.
        # These are in DEG/S because D1 uses degrees on-wire.
        self.max_joint_speed_deg_s = float(getattr(config, "max_joint_speed_deg_s", 20.0))   # joints 0-5
        self.max_gripper_speed_deg_s = float(getattr(config, "max_gripper_speed_deg_s", 40.0))  # joint 6
        # EMA low-pass smoothing on commanded position (deg). 0 disables.
        self.ema_alpha = float(getattr(config, "ema_alpha", 0.15))
        self.ema_alpha = float(np.clip(self.ema_alpha, 0.0, 1.0))
        # Warmup blending time (seconds): blend from measured pose -> policy pose.
        self.warmup_seconds = float(getattr(config, "warmup_seconds", 1.5))
        self.warmup_seconds = max(0.0, self.warmup_seconds)


        # ========================= Z1 -> D1 mapping (ONLY mapping change) =========================
        # dual/Z1 action convention (7D): [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper]
        # D1 expects 7D: [J0..J6]. We support:
        #   - reorder (z1_to_d1_index)
        #   - sign flip (d1_joint_sign)
        #   - relative mapping (use_relative_mapping): drive D1 by policy deltas to avoid zero-point mismatch
        self.use_relative_mapping = bool(getattr(config, "use_relative_mapping", True))

        self.z1_to_d1_index = np.asarray(
            getattr(config, "z1_to_d1_index", [0, 1, 2, 3, 4, 5, 6]),
            dtype=np.int32,
        ).reshape(-1)
        if self.z1_to_d1_index.shape[0] != 7:
            log_warning("[D1_ArmController] z1_to_d1_index must be len=7; fallback identity.")
            self.z1_to_d1_index = np.asarray([0, 1, 2, 3, 4, 5, 6], dtype=np.int32)

        self.d1_joint_sign = np.asarray(
            getattr(config, "d1_joint_sign", [1, 1, 1, 1, 1, 1, 1]),
            dtype=np.float32,
        ).reshape(-1)
        if self.d1_joint_sign.shape[0] != 7:
            log_warning("[D1_ArmController] d1_joint_sign must be len=7; fallback all +1.")
            self.d1_joint_sign = np.ones((7,), dtype=np.float32)

        self.d1_delta_gain = np.asarray(
            getattr(config, "d1_delta_gain", [1, 1, 1, 1, 1, 1, 1]),
            dtype=np.float32,
        ).reshape(-1)
        if self.d1_delta_gain.shape[0] != 7:
            log_warning("[D1_ArmController] d1_delta_gain must be len=7; fallback all 1.")
            self.d1_delta_gain = np.ones((7,), dtype=np.float32)

        self.d1_offset_rad = np.asarray(
            getattr(config, "d1_offset_rad", [0, 0, 0, 0, 0, 0, 0]),
            dtype=np.float32,
        ).reshape(-1)
        if self.d1_offset_rad.shape[0] != 7:
            self.d1_offset_rad = np.zeros((7,), dtype=np.float32)

        # mapping baselines (locked on first received policy action)
        self._policy0_mapped: Optional[np.ndarray] = None
        self._d1_base: Optional[np.ndarray] = None
        # ----- internal state -----
        init_pose = np.zeros((self._num_joints,), dtype=np.float32)
        if getattr(config, "init_pose", None) is not None:
            try:
                p = np.asarray(config.init_pose, dtype=np.float32).reshape(-1)
                if p.shape[0] == self._num_joints:
                    init_pose = p.copy()
            except Exception:
                pass

        self._last_q_measured = init_pose.copy()
        self._last_q_measured_ts = 0.0

        self.q_target = init_pose.copy()
        self.tauff_target = np.zeros_like(init_pose)
        self.time_target: Optional[float] = None  # perf_counter timebase (see UnitreeRobot.send_action)
        self.arm_cmd: Optional[str] = None

        t0 = time.monotonic()
        self.pose_interp = JointTrajectoryInterpolator(times=[t0], joint_positions=[init_pose.copy()])
        self._stop_pose = init_pose.copy()
        self._stop_motion = False


        # last sent command (deg), for per-step speed limiting + EMA
        self._last_sent_deg = np.zeros((self._num_joints,), dtype=np.float32)
        self._ema_deg = None  # type: Optional[np.ndarray]
        self._warmup_t0 = None  # type: Optional[float]
        # ----- threads -----
        self._stop_event = threading.Event()
        self._ctrl_thread: Optional[threading.Thread] = None
        self._fb_thread: Optional[threading.Thread] = None

        # seq counter for JSON
        self._seq_counter = int(time.time() * 1000) & 0xFFFFFFFF

    # ========================= properties =========================
    @property
    def motor_names(self) -> list[str]:
        return self._motor_names

    # ========================= low-level io =========================
    def _assert_connected(self):
        if not self.is_connected or self.sock is None or self.f is None:
            raise RobotDeviceNotConnectedError("D1_ArmController is not connected.")

    def _send_line(self, line: str) -> None:
        """Send one line (must include trailing \n)."""
        self._assert_connected()
        if not line.endswith("\n"):
            line += "\n"
        data = line.encode("utf-8")
        with self.io_lock:
            try:
                self.f.write(data)  # type: ignore
            except Exception as e:
                raise ConnectionError(f"bridge write failed: {e}")

    def _read_line(self, timeout: float = 1.0) -> str:
        """Read one line with socket timeout."""
        self._assert_connected()
        with self.io_lock:
            try:
                self.sock.settimeout(timeout)
                raw = self.f.readline()  # type: ignore
            except Exception as e:
                raise ConnectionError(f"bridge read failed: {e}")
        if not raw:
            raise ConnectionError("bridge returned empty line")
        if isinstance(raw, bytes):
            return raw.decode("utf-8", errors="ignore").strip()
        return str(raw).strip()

    def _next_seq(self) -> int:
        self._seq_counter = (self._seq_counter + 1) & 0xFFFFFFFF
        return self._seq_counter

    def _send_json_cmd(self, obj: dict) -> None:
        self._send_line("CMD " + json.dumps(obj, separators=(",", ":")))

    # ========================= bridge protocol =========================
    def _read_q_from_bridge(self, timeout: float = 1.0) -> np.ndarray:
        """GET_Q -> Q deg0 ... deg6, returns radians (7D)."""
        self._send_line("GET_Q")
        line = self._read_line(timeout=timeout)
        if not line.startswith("Q "):
            raise RuntimeError(f"unexpected bridge reply: {line}")
        parts = line.split()
        if len(parts) != 1 + self._num_joints:
            raise RuntimeError(f"bad Q length: {len(parts)-1} (expect {self._num_joints}), line={line}")
        deg = np.array([float(x) for x in parts[1:]], dtype=np.float32)
        rad = np.deg2rad(deg)
        return self._clip_q(rad)

    def _send_q_cmd(self, q_rad: np.ndarray, smooth_mode: int = 0) -> None:
        """Send funcode=2 (all joints) in degrees."""
        q = np.asarray(q_rad, dtype=np.float32).reshape(-1)
        if q.shape[0] != self._num_joints:
            raise ValueError(f"q_cmd dim mismatch: expect {self._num_joints}, got {q.shape[0]}")
        q = self._map_z1_to_d1(q)
        deg = np.rad2deg(q)
        payload = {
            "seq": int(self._next_seq()),
            "address": 1,
            "funcode": 2,
            "data": {"mode": int(smooth_mode)},
        }
        for i in range(self._num_joints):
            payload["data"][f"angle{i}"] = float(deg[i])
        self._send_json_cmd(payload)

    # ========================= safety / limits =========================
    def _clip_q(self, q_rad: np.ndarray) -> np.ndarray:
        q = np.asarray(q_rad, dtype=np.float32).reshape(-1)
        if q.shape[0] != self._num_joints:
            return q
        return np.clip(q, self.joint_rad_min, self.joint_rad_max)

    
    # ========================= mapping: policy(Z1)->D1 =========================
    def _map_z1_to_d1(self, q_policy_7_rad: np.ndarray) -> np.ndarray:
        \"\"\"Map 7D policy action (Z1 order) -> 7D D1 command (rad).

        This file version changes ONLY mapping. All other control logic is untouched.
        Defaults:
          - reorder via z1_to_d1_index
          - relative mapping (recommended): D1 follows policy deltas from the first frame.
        \"\"\"
        q_policy = np.asarray(q_policy_7_rad, dtype=np.float32).reshape(-1)
        if q_policy.shape[0] != 7:
            raise ValueError(f\"_map_z1_to_d1 expects 7 dims, got {q_policy.shape[0]}\")
        # reorder into D1 joint order
        q_mapped = np.zeros((7,), dtype=np.float32)
        for d1_i in range(7):
            src = int(self.z1_to_d1_index[d1_i])
            src = 0 if src < 0 else (6 if src > 6 else src)
            q_mapped[d1_i] = float(q_policy[src])

        # Ensure D1 base pose exists (takeover pose)
        with self.ctrl_lock:
            q_meas = self._last_q_measured.copy()
        if self._d1_base is None:
            self._d1_base = q_meas.copy()

        if self.use_relative_mapping:
            if self._policy0_mapped is None:
                # lock baselines on first policy frame, and do NOT jump
                self._policy0_mapped = q_mapped.copy()
                self._d1_base = q_meas.copy()
                return self._clip_q(q_meas)

            delta = (q_mapped - self._policy0_mapped) * self.d1_delta_gain
            delta = delta * self.d1_joint_sign
            q_cmd = self._d1_base + delta + self.d1_offset_rad
            return self._clip_q(q_cmd)

        # absolute mode (not recommended unless zero-points already aligned)
        q_cmd = (q_mapped * self.d1_joint_sign) + self.d1_offset_rad
        return self._clip_q(q_cmd)

# ========================= public API =========================

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError("D1_ArmController already connected")

        log_info(f"[D1_ArmController] connect bridge tcp={self.bridge_host}:{self.bridge_port}")
        try:
            self.sock = socket.create_connection((self.bridge_host, self.bridge_port), timeout=3.0)
            # unbuffered binary file
            self.f = self.sock.makefile("rwb", buffering=0)
        except Exception as e:
            self.sock = None
            self.f = None
            raise ConnectionError(
                f"connect to bridge failed: {e}. "
                f"Is d1_bridge running on {self.bridge_host}:{self.bridge_port}?"
            )

        self.is_connected = True

        # 1) read current q first (avoid jump-to-zero)
        q0 = None
        for _ in range(30):
            try:
                q0 = self._read_q_from_bridge(timeout=0.5)
                break
            except Exception:
                time.sleep(0.05)
        if q0 is None:
            q0 = np.zeros((self._num_joints,), dtype=np.float32)
            log_warning("[D1_ArmController] cannot get initial q from bridge; fallback zeros (be careful)")

        with self.ctrl_lock:
            self._last_q_measured = q0.copy()
            self._last_q_measured_ts = time.monotonic()
            self.q_target = q0.copy()
            self._stop_pose = q0.copy()
            self.pose_interp = JointTrajectoryInterpolator(times=[time.monotonic()], joint_positions=[q0.copy()])

            # reset mapping baselines on (re)connect
            self._policy0_mapped = None
            self._d1_base = q0.copy()


        # Initialize smoothing / rate-limit states
        self._last_sent_deg = np.rad2deg(q0).astype(np.float32)
        self._ema_deg = self._last_sent_deg.copy()
        self._warmup_t0 = time.monotonic()

        # 2) power + enable (best-effort, send both "1" and "80000" to match doc variants)
        try:
            self.set_power(True)
            time.sleep(0.05)
            self.enable_all()
            time.sleep(0.05)
        except Exception as e:
            log_warning(f"[D1_ArmController] power/enable failed (ignored): {e}")

        # 3) start threads
        self._stop_event.clear()
        self._ctrl_thread = threading.Thread(target=self._ctrl_loop, daemon=True)
        self._fb_thread = threading.Thread(target=self._feedback_poll_loop, daemon=True)
        self._ctrl_thread.start()
        self._fb_thread.start()

        # 4) send one hold command (helps some firmwares to latch)
        try:
            self._send_q_cmd(q0, smooth_mode=0)
        except Exception:
            pass

        log_success("[D1_ArmController] connected")

    def disconnect(self):
        if not self.is_connected:
            return

        self._stop_event.set()
        try:
            if self._ctrl_thread is not None:
                self._ctrl_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._fb_thread is not None:
                self._fb_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if self.f is not None:
                self.f.close()  # type: ignore
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
        log_info("[D1_ArmController] disconnected")

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass

    # -------- state read --------
    def read_current_motor_q(self) -> np.ndarray:
        return self.read_current_arm_q()

    def read_current_arm_q(self) -> np.ndarray:
        # prefer cached
        now = time.monotonic()
        with self.ctrl_lock:
            q = self._last_q_measured.copy()
            age = now - float(self._last_q_measured_ts)
        # refresh if stale
        if age > 0.5:
            try:
                q2 = self._read_q_from_bridge(timeout=0.5)
                with self.ctrl_lock:
                    self._last_q_measured = q2.copy()
                    self._last_q_measured_ts = time.monotonic()
                q = q2
            except Exception:
                pass
        return q

    def read_current_arm_dq(self) -> np.ndarray:
        return np.zeros((self._num_joints,), dtype=np.float32)

    # -------- command write (called by UnitreeRobot.send_action) --------
    def write_arm(
        self,
        q_target: Union[list[float], np.ndarray],
        tauff_target: Union[list[float], np.ndarray, None] = None,
        time_target: float | None = None,
        cmd_target: str | None = None,
    ):
        q = np.asarray(q_target, dtype=np.float32).reshape(-1)
        # UnifolM dual checkpoints sometimes output higher-dim actions (e.g., 16D).
        # For D1 we follow Z1GripperArmJointIndex convention: take the first 7 dims.
        if q.shape[0] > self._num_joints:
            q = q[: self._num_joints]
        if q.shape[0] != self._num_joints:
            raise ValueError(f"D1 write_arm expects >= {self._num_joints} dims (will take first {self._num_joints}), got {q.shape[0]}")
        q = self._clip_q(q)
        with self.ctrl_lock:
            self.q_target = q
            if tauff_target is not None:
                self.tauff_target = np.asarray(tauff_target, dtype=np.float32).reshape(-1)
            self.time_target = time_target
            self.arm_cmd = cmd_target
            # clear stop_motion when a new command comes
            self._stop_motion = False

    # -------- optional helpers --------
    def stop_motion(self, reason: str = ""):
        if reason:
            log_warning(f"[D1_ArmController] stop_motion: {reason}")
        with self.ctrl_lock:
            self._stop_motion = True

    # ========================= D1 service commands =========================
    def set_power(self, on: bool):
        payload = {
            "seq": int(self._next_seq()),
            "address": 1,
            "funcode": 6,
            "data": {"power": 1 if on else 0},
        }
        self._send_json_cmd(payload)

    def set_all_damping_raw(self, mode: int):
        """funcode=5. D1 文档里有两种写法：mode=0/1 或 mode=0~80000。
        这里不做假设，直接下发整数。
        """
        payload = {
            "seq": int(self._next_seq()),
            "address": 1,
            "funcode": 5,
            "data": {"mode": int(mode)},
        }
        self._send_json_cmd(payload)

    def enable_all(self):
        # 先发 1（兼容 mode=0/1 的固件），再发 80000（兼容 mode=0~80000 的固件）
        try:
            self.set_all_damping_raw(1)
            time.sleep(0.02)
        except Exception:
            pass
        self.set_all_damping_raw(80000)

    def disable_all(self):
        self.set_all_damping_raw(0)

    # ========================= internal loops =========================
    def _ctrl_loop(self):
        """Control loop: 10Hz by default."""
        last_waypoint_time = None
        while not self._stop_event.is_set():
            t0 = time.perf_counter()
            try:
                with self.ctrl_lock:
                    target = self.q_target.copy()
                    arm_cmd = self.arm_cmd
                    t_target_pc = self.time_target
                    stop_motion = self._stop_motion
                    stop_pose = self._stop_pose.copy()

                # Convert perf_counter timebase -> monotonic timebase
                arm_time_target = None
                if t_target_pc is not None:
                    arm_time_target = time.monotonic() - time.perf_counter() + float(t_target_pc)

                t_now = time.monotonic()
                if self.pose_interp is None:
                    self.pose_interp = JointTrajectoryInterpolator(times=[t_now], joint_positions=[stop_pose])

                if stop_motion:
                    # hold
                    self.pose_interp = JointTrajectoryInterpolator(times=[t_now], joint_positions=[stop_pose])
                    last_waypoint_time = self.pose_interp.times[-1]
                else:
                    # update interpolator
                    if arm_cmd == "drive_to_waypoint":
                        self.pose_interp = self.pose_interp.drive_to_waypoint(
                            pose=target,
                            time=t_now + 0.8,
                            curr_time=t_now,
                            max_pos_speed=self.max_pos_speed,
                        )
                        last_waypoint_time = self.pose_interp.times[-1]
                    elif arm_cmd == "schedule_waypoint":
                        # schedule using external time target
                        t_insert = arm_time_target if arm_time_target is not None else (t_now + 0.8)
                        self.pose_interp = self.pose_interp.schedule_waypoint(
                            pose=target,
                            time=t_insert,
                            max_pos_speed=self.max_pos_speed,
                            curr_time=t_now,
                            last_waypoint_time=last_waypoint_time,
                        )
                        last_waypoint_time = self.pose_interp.times[-1]
                    else:
                        # default: just hold at latest target
                        self.pose_interp = self.pose_interp.drive_to_waypoint(
                            pose=target,
                            time=t_now + self.control_dt,
                            curr_time=t_now,
                            max_pos_speed=self.max_pos_speed,
                        )
                        last_waypoint_time = self.pose_interp.times[-1]

                q_des = self.pose_interp(t_now)
                q_des = self._clip_q(q_des)

                # -------- warmup blend (avoid jump on takeover) --------
                if self.warmup_seconds > 1e-6 and self._warmup_t0 is not None:
                    w = (t_now - float(self._warmup_t0)) / float(self.warmup_seconds)
                    w = float(np.clip(w, 0.0, 1.0))
                    with self.ctrl_lock:
                        q_meas = self._last_q_measured.copy()
                    q_des = (1.0 - w) * q_meas + w * q_des

                # -------- EMA smoothing (deg) --------
                cmd_deg = np.rad2deg(q_des).astype(np.float32)
                if self.ema_alpha > 0.0:
                    if self._ema_deg is None:
                        self._ema_deg = cmd_deg.copy()
                    else:
                        self._ema_deg = (1.0 - self.ema_alpha) * self._ema_deg + self.ema_alpha * cmd_deg
                    cmd_deg = self._ema_deg

                # -------- per-step speed limiting (deg/step) --------
                max_step = np.full((self._num_joints,), self.max_joint_speed_deg_s * self.control_dt, dtype=np.float32)
                if self._num_joints >= 7:
                    max_step[6] = self.max_gripper_speed_deg_s * self.control_dt
                # clamp to reasonable minimum/maximum to avoid misconfig
                max_step = np.clip(max_step, 0.1, 30.0).astype(np.float32)

                last_deg = self._last_sent_deg.astype(np.float32)
                delta = cmd_deg - last_deg
                delta = np.clip(delta, -max_step, +max_step)
                send_deg = last_deg + delta

                # enforce joint limits again (deg), then send
                send_rad = self._clip_q(np.deg2rad(send_deg))
                self._send_q_cmd(send_rad, smooth_mode=0)

                # update last sent
                self._last_sent_deg = np.rad2deg(send_rad).astype(np.float32)
                q_cmd = send_rad

                with self.ctrl_lock:
                    self._stop_pose = q_cmd.copy()

            except Exception as e:
                log_error(f"[D1_ArmController] ctrl loop exception: {e}")
                try:
                    self.stop_motion(reason=f"ctrl_exception: {e}")
                except Exception:
                    pass

            elapsed = time.perf_counter() - t0
            time.sleep(max(0.0, self.control_dt - elapsed))

    def _feedback_poll_loop(self):
        """Poll GET_Q periodically to keep a fresh q cache for capture_observation."""
        while not self._stop_event.is_set():
            t0 = time.perf_counter()
            try:
                q = self._read_q_from_bridge(timeout=1.0)
                with self.ctrl_lock:
                    self._last_q_measured = q.copy()
                    self._last_q_measured_ts = time.monotonic()
            except Exception:
                pass
            elapsed = time.perf_counter() - t0
            time.sleep(max(0.0, self.feedback_dt - elapsed))

    # ========================= IK placeholders =========================
    def arm_ik(self, *args, **kwargs):
        return None

    def arm_fk(self, *args, **kwargs):
        return None

    def go_start(self):
        # 不强制 go_start（D1 上电后可能不在安全位姿）
        return

    def go_home(self):
        return