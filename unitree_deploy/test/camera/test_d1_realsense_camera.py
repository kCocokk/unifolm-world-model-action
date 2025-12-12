# unitree_deploy/test/camera/test_d1_realsense_camera.py

import time

from unitree_deploy.robot_devices.cameras.configs import IntelRealSenseCameraConfig
from unitree_deploy.robot_devices.cameras.intelrealsense import IntelRealSenseCamera
from unitree_deploy.utils.rich_logger import log_info, log_success, log_error


def main():
    # ⚠️ 请把 serial_number 改成你自己的设备序列号（用 rs-enumerate-devices 查）
    cfg = IntelRealSenseCameraConfig(
        serial_number="YOUR_D435_SERIAL",
        width=640,
        height=480,
        fps=30,
    )

    log_info("[test_d1_realsense_camera] Initializing camera ...")

    try:
        cam = IntelRealSenseCamera(cfg)
    except Exception as e:
            log_error(f"[test_d1_realsense_camera] Failed to init camera: {e}")
            return

    log_success("[test_d1_realsense_camera] Camera initialized.")

    for i in range(30):
        frame = cam.read()
        if frame is None:
            log_error("[test_d1_realsense_camera] Empty frame, 请检查 RealSense 连接。")
            break

        rgb = frame.get("rgb", None)
        depth = frame.get("depth", None)

        log_info(
            f"[test_d1_realsense_camera] Frame {i}: "
            f"rgb shape={None if rgb is None else rgb.shape}, "
            f"depth shape={None if depth is None else depth.shape}"
        )
        time.sleep(0.1)

    log_success("[test_d1_realsense_camera] DONE.")


if __name__ == "__main__":
    main()
