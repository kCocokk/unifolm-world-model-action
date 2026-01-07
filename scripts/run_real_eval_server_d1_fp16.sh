#!/usr/bin/env bash
set -euo pipefail

# =====================
# D1 real inference server (FP16) launcher
# =====================
# This starts FastAPI/uvicorn inside scripts/evaluation/real_eval_server_fp16.py.
# Intended for 8GB GPUs (e.g. RTX 4060). If you still OOM, reduce --height/--width,
# or reduce --video_length / --ddim_steps.

cd /home/coco/model/unifolm-world-model-action

# ---- paths (override by env if you want) ----
CKPT=${CKPT:-/home/coco/model/unifolm_checkpoints/Dual/unifolm_wma_dual.ckpt}
CONFIG=${CONFIG:-configs/inference/world_model_decision_making.yaml}
SAVEDIR=${SAVEDIR:-/tmp/unifolm_server}

# ---- memory / perf knobs ----
export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
export PYTORCH_CUDA_ALLOC_CONF=${PYTORCH_CUDA_ALLOC_CONF:-max_split_size_mb:128}

# If you plan to run on another machine and call it from your laptop, set HOST=0.0.0.0
HOST=${HOST:-127.0.0.1}
PORT=${PORT:-8000}

# IMPORTANT:
# The original real_eval_server.py hardcodes server.run(host='127.0.0.1', port=8000)
# If your real_eval_server_fp16.py also hardcodes that, either:
#   1) temporarily edit the last line to: server.run(host='${HOST}', port=${PORT})
# or
#   2) add argparse args for host/port.

python3 -u scripts/evaluation/real_eval_server_fp16.py \
  --seed 123 \
  --ckpt_path "${CKPT}" \
  --config "${CONFIG}" \
  --savedir "${SAVEDIR}" \
  --bs 1 \
  --height 320 --width 512 \
  --unconditional_guidance_scale 1.0 \
  --ddim_steps 16 \
  --ddim_eta 1.0 \
  --video_length 16 \
  --frame_stride 2 \
  --timestep_spacing 'uniform_trailing' \
  --guidance_rescale 0.7 \
  --perframe_ae \
  2>&1 | tee "${SAVEDIR}/server.log"
