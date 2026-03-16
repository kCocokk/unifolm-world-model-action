#!/usr/bin/env bash
set -euo pipefail

cd /data6/user24215463/unifolm-world-model-action

CKPT=${CKPT:-/data6/user24215463/unifolm_checkpoints/D1_Base_FT/last.ckpt}
CONFIG=${CONFIG:-configs/inference/world_model_decision_making_d1.yaml}
SAVEDIR=${SAVEDIR:-/data6/user24215463/unifolm_checkpoints/server_d1_logs}
HOST=${HOST:-127.0.0.1}
PORT=${PORT:-8000}

mkdir -p "${SAVEDIR}"
export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
export PYTORCH_CUDA_ALLOC_CONF=${PYTORCH_CUDA_ALLOC_CONF:-max_split_size_mb:128}

python3 -u scripts/evaluation/real_eval_server_fp16.py   --seed 123   --ckpt_path "${CKPT}"   --config "${CONFIG}"   --savedir "${SAVEDIR}"   --bs 1   --height 320 --width 512   --unconditional_guidance_scale 1.0   --ddim_steps 16   --ddim_eta 1.0   --video_length 16   --frame_stride 2   --timestep_spacing 'uniform_trailing'   --guidance_rescale 0.7   --perframe_ae   --fp16   2>&1 | tee "${SAVEDIR}/server.log"
