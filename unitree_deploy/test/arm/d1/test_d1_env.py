# unitree_deploy/test/arm/d1/test_d1_env.py

import time
import numpy as np
import torch

from unitree_deploy.robot.robot_utils import make_robot
from unitree_deploy.utils.rich_logger import log_info, log_success, log_error


def main():
    robot_type = "d1_realsense"
    log_info(f"[test_d1_env] Creating robot '{robot_type}' ...")

    robot = make_robot(robot_type)

    try:
        log_info("[test_d1_env] Connecting robot ...")
        robot.connect()
        log_success("[test_d1_env] Robot connected.")

        # 读取一帧 observation
        obs = robot.capture_observation()
        log_info(f"[test_d1_env] Obs keys: {list(obs.keys())}")

        # 尝试从常见 key 中取状态
        state = None
        for k in ["observation.state", "state", "q"]:
            if k in obs:
                state = obs[k]
                log_info(f"[test_d1_env] Found state in key '{k}'")
                break

        # 尝试从常见 key 中取图像
        image = None
        for k in ["observation.image", "observation.rgb", "image", "rgb"]:
            if k in obs:
                image = obs[k]
                log_info(f"[test_d1_env] Found image in key '{k}'")
                break

        if state is None:
            log_error(f"[test_d1_env] Cannot find state in obs keys: {list(obs.keys())}")
            return

        if isinstance(state, torch.Tensor):
            state_np = state.detach().cpu().numpy()
        else:
            state_np = np.asarray(state)

        log_info(f"[test_d1_env] state shape={state_np.shape}")

        if image is not None:
            shape = getattr(image, "shape", "n/a")
            log_info(f"[test_d1_env] image type={type(image)}, shape={shape}")

        # 对关节施加小幅动作：前 6 维 +5°
        action = state_np.copy()
        n = min(6, action.shape[-1])
        action[..., :n] += np.deg2rad(5.0)

        log_info("[test_d1_env] Sending small joint command (+5deg on first 6 joints) ...")
        t_command = time.monotonic() + 0.1
        action_tensor = torch.from_numpy(action.astype(np.float32))
        robot.send_action(action_tensor, t_command_target=t_command)

        time.sleep(1.0)
        log_success("[test_d1_env] DONE, 请确认 D1 有小幅运动且图像/状态能正常获取。")

    finally:
        try:
            robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
