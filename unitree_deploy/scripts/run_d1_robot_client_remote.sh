#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

# -------- D1 local hardware settings --------
export D1_ROBOT_TYPE="${D1_ROBOT_TYPE:-d1_opencv_slave}"
export D1_BRIDGE_HOST="${D1_BRIDGE_HOST:-127.0.0.1}"
export D1_BRIDGE_PORT="${D1_BRIDGE_PORT:-5556}"

export D1_CAM_DEVICE="${D1_CAM_DEVICE:-/dev/video2}"
export D1_CAMERA_WIDTH="${D1_CAMERA_WIDTH:-640}"
export D1_CAMERA_HEIGHT="${D1_CAMERA_HEIGHT:-480}"
export D1_CAMERA_FPS="${D1_CAMERA_FPS:-30}"

# -------- remote inference server --------
# If you use SSH tunnel: 127.0.0.1:8000
export UNIFOLM_SERVER_HOST="${UNIFOLM_SERVER_HOST:-127.0.0.1}"
export UNIFOLM_SERVER_PORT="${UNIFOLM_SERVER_PORT:-8000}"

LANG="${LANG:-Pick and place}"
OUTDIR="${OUTDIR:-/tmp/d1_unifolm_rollouts}"
N="${N:-1}"

echo "[D1] bridge=${D1_BRIDGE_HOST}:${D1_BRIDGE_PORT} cam=${D1_CAM_DEVICE} ${D1_CAMERA_WIDTH}x${D1_CAMERA_HEIGHT}@${D1_CAMERA_FPS}"
echo "[Policy] server=http://${UNIFOLM_SERVER_HOST}:${UNIFOLM_SERVER_PORT}"
echo "[Run] language='${LANG}' outdir=${OUTDIR} rollouts=${N}"

python3 -u unitree_deploy/scripts/robot_client.py \
  --robot_type "${D1_ROBOT_TYPE}" \
  --language_instruction "${LANG}" \
  --num_rollouts_planned "${N}" \
  --output_dir "${OUTDIR}" \
  --control_freq 30 \
  --observation_horizon 2 \
  --action_horizon 16 \
  --exe_steps 16
