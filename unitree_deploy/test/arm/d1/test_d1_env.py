# unitree_deploy/test/arm/d1/test_d1_env.py
#
# 目的：验证 UnitreeRobot 的“环境式接口”：
#   - connect()
#   - capture_observation() 读取 state（以及可选的图像）
#   - send_action() 下发 action（一次小幅动作）
#
# 运行：
#   python unitree_deploy/test/arm/d1/test_d1_env.py
#
# 可选：
#   export D1_USE_CAMERA=1          # 尝试用 d1_realsense（如果没相机/没装驱动，会自动回退成只用机械臂）
#   export D1_REALSENSE_SERIAL=...  # 你的 RealSense 序列号（可不设，尝试选择第一台）

import os
import time
import numpy as np
import torch

from unitree_deploy.robot.robot_utils import make_robot
from unitree_deploy.utils.rich_logger import log_info, log_success, log_error


def _print_obs_keys(obs: dict):
    keys = sorted(list(obs.keys()))
    log_info(f"[test_d1_env] obs keys: {keys}")


def main():
    use_cam = os.getenv("D1_USE_CAMERA", "0").strip() in ("1", "true", "True")
    robot_type = os.getenv("D1_ROBOT_TYPE", "").strip() or ("d1_realsense" if use_cam else "d1_only_arm")
    log_info(f"[test_d1_env] Creating robot '{robot_type}' ...")
    robot = make_robot(robot_type)

    ok = False
    try:
        log_info("[test_d1_env] Connecting ...")
        robot.connect()
        log_success("[test_d1_env] Connected.")

        obs = robot.capture_observation()
        _print_obs_keys(obs)

        state = obs.get("observation.state", None)
        if state is None:
            raise RuntimeError("observation.state not found in obs")
        log_info(f"[test_d1_env] state shape={tuple(state.shape)} dtype={state.dtype}")

        expected = 0
        for arm_name in robot.arm:
            expected += len(robot.arm[arm_name].motor_names)
        for ee_name in robot.endeffector:
            expected += len(robot.endeffector[ee_name].motor_names)

        action = state.detach().cpu().numpy().copy()
        if action.size != expected:
            action = action[:expected].copy()

        action[0] = float(np.clip(action[0] + 0.05, -2.35, 2.35))
        action_tensor = torch.from_numpy(action).to(dtype=torch.float32).contiguous()

        t_command = time.monotonic() + 0.3
        log_info(f"[test_d1_env] send_action, dim={action_tensor.numel()}, t_command_target={t_command:.3f}")
        robot.send_action(action_tensor, t_command_target=t_command)

        time.sleep(1.2)
        log_success("[test_d1_env] DONE. 请确认机械臂有小幅运动；若接了相机，也应能看到 observation.images.*")
        ok = True

    except Exception as e:
        log_error(f"[test_d1_env] FAILED: {e}")
        ok = False

    finally:
        try:
            robot.disconnect()
            log_info("[test_d1_env] Disconnected OK")
        except Exception as e:
            log_error(f"[test_d1_env] Disconnect error: {e}")

        # ✅ 给 C++/后台线程一点时间收尾，减少 core dump
        time.sleep(0.3)

    # ✅ 不 raise，让进程正常退出
    if not ok:
        raise SystemExit(1)

if __name__ == "__main__":
    main()