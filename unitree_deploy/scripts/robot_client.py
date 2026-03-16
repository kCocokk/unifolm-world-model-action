import argparse
import os
import time
import cv2
import numpy as np
import torch
import tqdm

from typing import Any, Deque, MutableMapping, OrderedDict
from collections import deque
from pathlib import Path

from unitree_deploy.real_unitree_env import make_real_env
from unitree_deploy.utils.eval_utils import (
    ACTTemporalEnsembler,
    LongConnectionClient,
    populate_queues,
)

# -----------------------------------------------------------------------------
# Network defaults (remote server supported by env)
# -----------------------------------------------------------------------------
os.environ["http_proxy"] = ""
os.environ["https_proxy"] = ""

HOST = os.environ.get("UNIFOLM_SERVER_HOST", "127.0.0.1")
PORT = int(os.environ.get("UNIFOLM_SERVER_PORT", "8000"))
BASE_URL = f"http://{HOST}:{PORT}"

# -----------------------------------------------------------------------------
# Robot defaults
# -----------------------------------------------------------------------------
# fmt: off
INIT_POSE = {
    'g1_dex1': np.array([0.10559805, 0.02726714, -0.01210221, -0.33341318, -0.22513399, -0.02627627, -0.15437093,  0.1273793 , -0.1674708 , -0.11544029, -0.40095493,  0.44332668,  0.11566751,  0.3936641, 5.4, 5.4], dtype=np.float32),
    'z1_dual_dex1_realsense': np.array([-1.0262332,  1.4281361, -1.2149128,  0.6473399, -0.12425245, 0.44945636,  0.89584476,  1.2593982, -1.0737865,  0.6672816, 0.39730102, -0.47400007, 0.9894176, 0.9817477 ], dtype=np.float32),
    'z1_realsense': np.array([-0.06940782, 1.4751548, -0.7554075, 1.0501366, 0.02931615, -0.02810347, -0.99238837], dtype=np.float32),

    # D1: 不建议上电就强行去某个姿态，这里用 None 表示跳过 warm-start 动作
    'd1_opencv_slave': None,
}

# Model side expects action/state dim=16 (uni action/state); for smaller robots, we pad.
ZERO_ACTION = {
    'g1_dex1': torch.zeros(16, dtype=torch.float32),
    'z1_dual_dex1_realsense': torch.zeros(14, dtype=torch.float32),
    'z1_realsense': torch.zeros(7, dtype=torch.float32),

    # D1: pad to 16 for server (it will map/mask internally)
    'd1_opencv_slave': torch.zeros(7, dtype=torch.float32),
}

CAM_KEY = {
    'g1_dex1': 'cam_right_high',
    'z1_dual_dex1_realsense': 'cam_high',
    'z1_realsense': 'cam_high',

    # D1 config camera name is cam_high
    'd1_opencv_slave': 'cam_high',
}
# fmt: on


def prepare_observation(args: argparse.Namespace, obs: Any) -> OrderedDict:
    """
    Convert a raw env observation into the model's expected input dict.
    - image: RGB uint8 -> torch (C,H,W)
    - state: keep the robot's native state dimension (D1 stays 7D)
    - action: zeros in the robot's native action dimension
    """
    bgr = obs.observation["images"][CAM_KEY[args.robot_type]]
    rgb_image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    qpos = obs.observation["qpos"]

    observation = {
        "observation.images.top": torch.from_numpy(rgb_image).permute(2, 0, 1),
        "observation.state": torch.from_numpy(np.asarray(qpos, dtype=np.float32)),
        "action": ZERO_ACTION[args.robot_type],
    }
    return OrderedDict(observation)


def _decode_action_for_robot(args: argparse.Namespace, action_1d: np.ndarray) -> np.ndarray:
    """
    Server returns uni-action (usually 16D after unnormalize).
    For D1 we only execute first 7 joints on the real robot.
    """
    action_1d = np.asarray(action_1d, dtype=np.float32).reshape(-1)
    if args.robot_type.startswith("d1"):
        return action_1d[:7].copy()
    return action_1d.copy()


def run_policy(
    args: argparse.Namespace,
    env: Any,
    client: LongConnectionClient,
    temporal_ensembler: ACTTemporalEnsembler,
    cond_obs_queues: MutableMapping[str, Deque[torch.Tensor]],
    output_dir: Path,
) -> None:
    """
    Single rollout loop:
        1) optional warm start (skip for D1),
        2) stream observations,
        3) fetch actions from the policy server,
        4) execute with temporal ensembling for smoother control.
    """
    init_pose = INIT_POSE.get(args.robot_type, None)
    if init_pose is not None:
        _ = env.step(init_pose)
        time.sleep(2.0)

    t = 0
    while True:
        obs = env.get_observation(t)

        obs_fmt = prepare_observation(args, obs)
        cond_obs_queues = populate_queues(cond_obs_queues, obs_fmt)

        pred_actions = client.predict_action(args.language_instruction, cond_obs_queues).unsqueeze(0)

        actions = temporal_ensembler.update(pred_actions[:, : args.action_horizon])[0]

        for n in range(args.exe_steps):
            action = actions[n].cpu().numpy()
            action_exec = _decode_action_for_robot(args, action)

            print(f">>> Exec => step {n} action_exec({action_exec.shape[0]}): {action_exec}", flush=True)
            print("---------------------------------------------")

            t1 = time.time()
            obs = env.step(action_exec)
            time.sleep(max(0, 1 / args.control_freq - time.time() + t1))
            t += 1

            if n < args.exe_steps - 1:
                obs_fmt = prepare_observation(args, obs)
                cond_obs_queues = populate_queues(cond_obs_queues, obs_fmt)


def run_eval(args: argparse.Namespace) -> None:
    print(f"[robot_client] Policy server: {BASE_URL}", flush=True)
    client = LongConnectionClient(BASE_URL)

    temporal_ensembler = ACTTemporalEnsembler(
        temporal_ensemble_coeff=0.01, chunk_size=args.action_horizon, exe_steps=args.exe_steps
    )
    temporal_ensembler.reset()

    cond_obs_queues = {
        "observation.images.top": deque(maxlen=args.observation_horizon),
        "observation.state": deque(maxlen=args.observation_horizon),
        "action": deque(maxlen=16),  # model predicts future 16 steps
    }

    env = make_real_env(
        robot_type=args.robot_type,
        dt=1 / args.control_freq,
    )
    env.connect()

    try:
        for episode_idx in tqdm.tqdm(range(0, args.num_rollouts_planned)):
            output_dir = Path(args.output_dir) / f"episode_{episode_idx:03d}"
            output_dir.mkdir(parents=True, exist_ok=True)
            run_policy(args, env, client, temporal_ensembler, cond_obs_queues, output_dir)
    finally:
        env.close()


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_type", type=str, default="g1_dex1", help="The type of the robot embodiment.")
    parser.add_argument("--action_horizon", type=int, default=16, help="Number of future actions to keep.")
    parser.add_argument("--exe_steps", type=int, default=16, help="Number of future actions to execute.")
    parser.add_argument("--observation_horizon", type=int, default=2, help="Number of most recent frames/states.")
    parser.add_argument(
        "--language_instruction",
        type=str,
        default="Pack black camera into box",
        help="Language instruction provided to the policy server.",
    )
    parser.add_argument("--num_rollouts_planned", type=int, default=10, help="The number of rollouts to run.")
    parser.add_argument("--output_dir", type=str, default="./results", help="Directory for saving results.")
    parser.add_argument("--control_freq", type=float, default=30, help="Low-level control frequency (Hz).")
    return parser


if __name__ == "__main__":
    args = get_parser().parse_args()
    run_eval(args)
