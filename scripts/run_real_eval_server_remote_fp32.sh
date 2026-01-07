#!/usr/bin/env bash
set -euo pipefail

# repo root = .../unifolm-world-model-action
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# -------- user-editable --------
CONFIG="${CONFIG:-${ROOT_DIR}/configs/inference/world_model_decision_making.yaml}"
CKPT_PATH="${CKPT_PATH:-/data6/user24215463/unifolm_checkpoints/Dual/unifolm_wma_dual.ckpt}"
SAVEDIR="${SAVEDIR:-/tmp/unifolm_server}"

# bind address/port (SSH tunnel: 127.0.0.1; public service: 0.0.0.0)
export UNIFOLM_SERVER_HOST="${UNIFOLM_SERVER_HOST:-127.0.0.1}"
export UNIFOLM_SERVER_PORT="${UNIFOLM_SERVER_PORT:-8000}"

# inference params (fp32 on server)
BS="${BS:-1}"
H="${H:-320}"
W="${W:-512}"
DDIM_STEPS="${DDIM_STEPS:-16}"
DDIM_ETA="${DDIM_ETA:-1.0}"
VIDEO_LENGTH="${VIDEO_LENGTH:-16}"
FRAME_STRIDE="${FRAME_STRIDE:-2}"
GUIDANCE_SCALE="${GUIDANCE_SCALE:-1.0}"
TIMESTEP_SPACING="${TIMESTEP_SPACING:-uniform_trailing}"
GUIDANCE_RESCALE="${GUIDANCE_RESCALE:-0.7}"
PERFRAME_AE="${PERFRAME_AE:-1}"
SEED="${SEED:-123}"

# -------- auto-fix data_dir (needed for normalizer/mapping init) --------
TMP_CONFIG="/tmp/unifolm_world_model_decision_making.runtime.yaml"
python3 - <<PY
from omegaconf import OmegaConf
import os

root = "${ROOT_DIR}"
cfg = OmegaConf.load("${CONFIG}")

example_dir = os.path.join(root, "examples", "world_model_interaction_prompts")
cfg.data.params.test.params.data_dir = example_dir

OmegaConf.save(cfg, "${TMP_CONFIG}")
print("Wrote runtime config:", "${TMP_CONFIG}")
print("data_dir ->", example_dir)
PY

echo "[Repo]   ROOT_DIR=${ROOT_DIR}"
echo "[Server] host=${UNIFOLM_SERVER_HOST} port=${UNIFOLM_SERVER_PORT}"
echo "[Server] ckpt=${CKPT_PATH}"
echo "[Server] savedir=${SAVEDIR}"
echo "[Server] config=${TMP_CONFIG}"

mkdir -p "${SAVEDIR}"

# -------- launch server --------
CMD=(python3 -u "${ROOT_DIR}/scripts/evaluation/real_eval_server.py"
  --seed "${SEED}"
  --ckpt_path "${CKPT_PATH}"
  --config "${TMP_CONFIG}"
  --savedir "${SAVEDIR}"
  --bs "${BS}" --height "${H}" --width "${W}"
  --unconditional_guidance_scale "${GUIDANCE_SCALE}"
  --ddim_steps "${DDIM_STEPS}"
  --ddim_eta "${DDIM_ETA}"
  --video_length "${VIDEO_LENGTH}"
  --frame_stride "${FRAME_STRIDE}"
  --timestep_spacing "${TIMESTEP_SPACING}"
  --guidance_rescale "${GUIDANCE_RESCALE}"
)

if [[ "${PERFRAME_AE}" == "1" ]]; then
  CMD+=(--perframe_ae)
fi

echo "Running: ${CMD[*]}"
"${CMD[@]}"
