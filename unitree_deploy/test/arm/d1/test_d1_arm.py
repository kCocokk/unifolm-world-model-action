import time
import numpy as np

from unitree_deploy.robot.robot_utils import make_robot
from unitree_deploy.utils.rich_logger import log_info, log_success, log_error


def main():
    robot_type = "d1_only_arm"  # 对应你在 robot_configs.py 里注册的 D1 机器人类型
    log_info(f"[test_d1_arm] Creating robot '{robot_type}' ...")
    robot = make_robot(robot_type)

    try:
        log_info("[test_d1_arm] Connecting to D1 (via d1_bridge) ...")
        robot.connect()
        log_success("[test_d1_arm] Robot connected.")

        if not robot.arm:
            log_error("[test_d1_arm] robot.arm 为空，请检查 RobotConfig 配置。")
            return

        arm_name, arm = list(robot.arm.items())[0]
        log_info(f"[test_d1_arm] Using arm '{arm_name}'.")

        # 读取当前关节角
        q = arm.read_current_arm_q()
        log_info(f"[test_d1_arm] current q (rad): {q}")

        # 在当前姿态基础上，让第 0 号关节 +10°
        q_cmd = np.array(q, dtype=float)
        q_cmd[0] += np.deg2rad(10.0)

        log_info("[test_d1_arm] Sending +10deg command on joint 0 ...")
        arm.write_arm(q_cmd)

        # 等 2 秒让机械臂动完
        time.sleep(2.0)

        q_new = arm.read_current_arm_q()
        log_info(f"[test_d1_arm] new q (rad): {q_new}")
        log_success("[test_d1_arm] DONE, 请观察 D1 底座关节是否有小幅旋转。")

    finally:
        try:
            robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
