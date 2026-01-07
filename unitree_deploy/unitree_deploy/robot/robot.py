import time
import torch

from unitree_deploy.robot.robot_configs import UnitreeRobotConfig
from unitree_deploy.robot_devices.arm.utils import make_arm_motors_buses_from_configs
from unitree_deploy.robot_devices.cameras.utils import make_cameras_from_configs
from unitree_deploy.robot_devices.endeffector.utils import (
    make_endeffector_motors_buses_from_configs,
)
from unitree_deploy.utils.rich_logger import log_success, log_warning, log_error, log_info


class UnitreeRobot:
    def __init__(self, config: UnitreeRobotConfig):
        self.config = config
        self.robot_type = self.config.type

        # NOTE: these are dicts
        self.cameras = make_cameras_from_configs(self.config.cameras)
        self.arm = make_arm_motors_buses_from_configs(self.config.arm)
        self.endeffector = make_endeffector_motors_buses_from_configs(self.config.endeffector)

        self.initial_data_received = True
        self.is_connected = False

    def connect(self):
        if (not self.arm) and (not self.endeffector) and (not self.cameras):
            raise ValueError("UnitreeRobot doesn't have any device to connect.")

        # 1) cameras first
        bad_cams = []
        for name, cam in list(self.cameras.items()):
            try:
                cam.connect()
                log_success(f"Connecting {name} cameras.")
            except Exception as e:
                log_warning(f"[UnitreeRobot] camera connect failed, disable camera={name}, err={e}")
                bad_cams.append(name)
        for name in bad_cams:
            self.cameras.pop(name, None)

        # 2) warmup camera reads
        bad_cams = set()
        for _ in range(20):
            for name, cam in list(self.cameras.items()):
                if name in bad_cams:
                    continue
                try:
                    cam.async_read()
                except Exception as e:
                    log_warning(f"[UnitreeRobot] camera warmup failed, disable camera={name}, err={e}")
                    bad_cams.add(name)
            time.sleep(1 / 30)
        for name in bad_cams:
            self.cameras.pop(name, None)

        # 3) arm
        for name in self.arm:
            self.arm[name].connect()
            log_success(f"Connecting {name} arm.")

        # 4) endeffector
        for name in self.endeffector:
            self.endeffector[name].connect()
            log_success(f"Connecting {name} endeffector.")

        time.sleep(0.5)
        self.is_connected = True
        log_success("All Device Connect Success!!!.✅")

    def capture_observation(self):
        """The returned observations do not have a batch dimension."""
        arm_state_list = []
        endeffector_state_list = []

        for arm_name in self.arm:
            arm_state = self.arm[arm_name].read_current_arm_q()
            arm_state_list.append(torch.from_numpy(arm_state))

        for endeffector_name in self.endeffector:
            endeffector_state = self.endeffector[endeffector_name].read_current_endeffector_q()
            endeffector_state_list.append(torch.from_numpy(endeffector_state))

        state = (
            torch.cat(arm_state_list + endeffector_state_list, dim=0)
            if arm_state_list or endeffector_state_list
            else torch.tensor([])
        )

        # Capture images
        images = {}
        for name, cam in list(self.cameras.items()):
            try:
                output = cam.async_read()
            except Exception as e:
                log_warning(f"[UnitreeRobot] camera read failed, skip camera={name}, err={e}")
                continue

            if isinstance(output, dict):
                images.update({k: torch.from_numpy(v) for k, v in output.items()})
            else:
                images[name] = torch.from_numpy(output)

        obs_dict = {"observation.state": state}
        for name, value in images.items():
            obs_dict[f"observation.images.{name}"] = value
        return obs_dict

    def send_action(self, action: torch.Tensor, t_command_target: float | None = None) -> torch.Tensor:
        """Send one action vector to robot (arm + endeffector). On error, do safe_stop."""
        try:
            if not isinstance(action, torch.Tensor):
                raise TypeError(f"action must be torch.Tensor, got {type(action)}")

            expected = 0
            for arm_name in self.arm:
                expected += len(self.arm[arm_name].motor_names)
            for ee_name in self.endeffector:
                expected += len(self.endeffector[ee_name].motor_names)

            if action.numel() != expected:
                raise ValueError(f"action dim mismatch: expect {expected}, got {action.numel()}")

            cmd_target = "drive_to_waypoint" if self.initial_data_received else "schedule_waypoint"

            # translate time target to perf_counter clock
            time_target = None
            if t_command_target is not None:
                time_target = t_command_target - time.monotonic() + time.perf_counter()

            from_idx_arm = 0
            to_idx_arm = 0
            action_sent_arm = []

            for arm_name in self.arm:
                to_idx_arm += len(self.arm[arm_name].motor_names)
                action_arm = action[from_idx_arm:to_idx_arm].detach().cpu().numpy()
                from_idx_arm = to_idx_arm

                action_sent_arm.append(torch.from_numpy(action_arm))
                self.arm[arm_name].write_arm(action_arm, time_target=time_target, cmd_target=cmd_target)

            from_idx_endeffector = to_idx_arm
            to_idx_endeffector = to_idx_arm
            action_endeffector_set = []

            for endeffector_name in self.endeffector:
                to_idx_endeffector += len(self.endeffector[endeffector_name].motor_names)
                action_endeffector = action[from_idx_endeffector:to_idx_endeffector].detach().cpu().numpy()
                from_idx_endeffector = to_idx_endeffector

                action_endeffector_set.append(torch.from_numpy(action_endeffector))
                self.endeffector[endeffector_name].write_endeffector(
                    action_endeffector, time_target=time_target, cmd_target=cmd_target
                )

            self.initial_data_received = False
            return torch.cat(action_sent_arm + action_endeffector_set, dim=0)

        except Exception as e:
            log_error(f"[UnitreeRobot] send_action failed: {e}")
            self.safe_stop(reason=f"send_action_failed: {e}")
            raise

    def safe_stop(self, reason: str = ""):
        """
        兜底：不要抛异常，尽可能让后台线程收尾，避免 python 退出 core dump。
        """
        if reason:
            log_warning(f"[UnitreeRobot] safe_stop: {reason}")

        # 这里不做 poweroff，只做断开（最稳）
        try:
            self.disconnect()
        except Exception:
            pass

    def disconnect(self):
        """
        正确的断开方式：self.arm / self.endeffector / self.cameras 都是 dict。
        """
        if getattr(self, "is_connected", False) is False:
            return

        # 先停控制设备，再停相机（通常更稳）
        # arm
        for name, arm_dev in list(getattr(self, "arm", {}).items()):
            try:
                if hasattr(arm_dev, "disconnect"):
                    arm_dev.disconnect()
            except Exception as e:
                log_warning(f"[UnitreeRobot] arm '{name}' disconnect error: {e}")

        # endeffector
        for name, ee_dev in list(getattr(self, "endeffector", {}).items()):
            try:
                if hasattr(ee_dev, "disconnect"):
                    ee_dev.disconnect()
            except Exception as e:
                log_warning(f"[UnitreeRobot] endeffector '{name}' disconnect error: {e}")

        # cameras
        for name, cam in list(getattr(self, "cameras", {}).items()):
            try:
                if hasattr(cam, "disconnect"):
                    cam.disconnect()
            except Exception as e:
                log_warning(f"[UnitreeRobot] camera '{name}' disconnect error: {e}")

        self.is_connected = False
        time.sleep(0.2)
        log_info("[UnitreeRobot] disconnected")

    def __del__(self):
        # 析构里绝不抛异常
        try:
            self.disconnect()
        except Exception:
            pass
