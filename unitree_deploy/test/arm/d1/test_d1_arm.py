# unitree_deploy/test/arm/d1/test_d1_arm.py
#
# 目的：不依赖摄像头，验证 D1（通过 d1_bridge）能连接、读关节、并做一个很小的关节运动。
#
# 运行：
#   python -m unitree_deploy.test.arm.d1.test_d1_arm
# 或：
#   python unitree_deploy/test/arm/d1/test_d1_arm.py
#
# 注意：确保 d1_bridge 已启动且能与机械臂通信。

import time
import numpy as np

from unitree_deploy.robot.robot_utils import make_robot
from unitree_deploy.utils.rich_logger import log_info, log_success, log_error


def main():
    robot_type = "d1_only_arm"
    log_info(f"[test_d1_arm] Creating robot '{robot_type}' ...")
    robot = make_robot(robot_type)

    try:
        log_info("[test_d1_arm] Connecting ...")
        robot.connect()
        log_success("[test_d1_arm] Connected.")

        # 读取当前关节
        arm = next(iter(robot.arm.values()))
        q = arm.read_current_arm_q().copy()
        log_info(f"[test_d1_arm] current q (rad+gripper_norm): {q}")

        # 做一个非常小的运动：J0 + 0.05 rad（约 3°）
        q_target = q.copy()
        q_target[0] = float(np.clip(q_target[0] + 0.05, -2.35, 2.35))
        log_info(f"[test_d1_arm] sending small motion, q_target[0]={q_target[0]:.4f} rad")
        arm.write_arm(q_target, cmd_target="schedule_waypoint", time_target=time.perf_counter() + 0.8)

        time.sleep(1.2)

        # 回到原位
        log_info("[test_d1_arm] return to original pose")
        arm.write_arm(q, cmd_target="schedule_waypoint", time_target=time.perf_counter() + 0.8)

        time.sleep(1.2)
        log_success("[test_d1_arm] DONE. 请确认 D1 J0 有小幅往返运动。")

    except Exception as e:
        log_error(f"[test_d1_arm] FAILED: {e}")
        raise

    finally:
        try:
            robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
