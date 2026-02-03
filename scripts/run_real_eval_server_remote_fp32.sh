model_name=testing
ckpt=/data6/user24215463/unifolm_checkpoints/Dual/unifolm_wma_dual.ckpt
config=/data6/user24215463/unifolm-world-model-action/configs/inference/world_model_decision_making.yaml
seed=123
res_dir="/tmp/unifolm_server"
datasets=(
    # "unitree_g1_pack_camera"
    "unitree_z1_stackbox"
)

# bind for SSH tunnel
export UNIFOLM_SERVER_HOST=127.0.0.1
export UNIFOLM_SERVER_PORT=8000

# (required) make runtime config with valid data_dir for normalizer/meta init
runtime_config=/tmp/unifolm_world_model_decision_making.runtime.yaml
python3 - <<'PY'
from omegaconf import OmegaConf
cfg = OmegaConf.load("/data6/user24215463/unifolm-world-model-action/configs/inference/world_model_decision_making.yaml")
cfg.data.params.test.params.data_dir = "/data6/user24215463/unifolm-world-model-action/examples/world_model_interaction_prompts"
OmegaConf.save(cfg, "/tmp/unifolm_world_model_decision_making.runtime.yaml")
print("Wrote:", "/tmp/unifolm_world_model_decision_making.runtime.yaml")
print("data_dir ->", cfg.data.params.test.params.data_dir)
PY

for dataset in "${datasets[@]}"; do
    CUDA_VISIBLE_DEVICES=2 python3 -u /data6/user24215463/unifolm-world-model-action/scripts/evaluation/real_eval_server.py \
    --seed ${seed} \
    --ckpt_path $ckpt \
    --config $runtime_config \
    --savedir "${res_dir}/${dataset}/${model_name}/videos" \
    --bs 1 --height 320 --width 512 \
    --unconditional_guidance_scale 1.0 \
    --ddim_steps 16 \
    --ddim_eta 1.0 \
    --video_length 16 \
    --frame_stride 2 \
    --timestep_spacing 'uniform_trailing' \
    --guidance_rescale 0.7 \
    --perframe_ae
done
