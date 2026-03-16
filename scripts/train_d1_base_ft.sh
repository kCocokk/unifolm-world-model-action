#!/usr/bin/env bash
set -euo pipefail

cd /data6/user24215463/unifolm-world-model-action
mkdir -p /data6/user24215463/unifolm_checkpoints/D1_Base_FT

export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}
export MASTER_ADDR=${MASTER_ADDR:-127.0.0.1}
export MASTER_PORT=${MASTER_PORT:-12366}

torchrun --nproc_per_node=1 scripts/trainer.py   --base configs/train/d1_base_ft.yaml   --train   --name D1_Base_FT   --logdir /data6/user24215463/unifolm_checkpoints   --devices 1   --total_gpus=1   lightning.trainer.num_nodes=1
