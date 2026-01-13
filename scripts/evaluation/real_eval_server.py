import argparse
import os
import sys
import warnings
import logging
import traceback
from datetime import datetime
from typing import Any, Dict, Optional, Tuple, List
from collections import OrderedDict

import torch
import torchvision
import imageio
import matplotlib.pyplot as plt
import uvicorn
from omegaconf import OmegaConf
from einops import rearrange, repeat
from pytorch_lightning import seed_everything
from torch import nn
from fastapi import FastAPI
from fastapi.responses import JSONResponse

from unifolm_wma.utils.utils import instantiate_from_config
from unifolm_wma.models.samplers.ddim import DDIMSampler

plt.switch_backend("agg")


def get_device_from_parameters(module: nn.Module) -> torch.device:
    """Get a module's device by checking one of its parameters."""
    return next(iter(module.parameters())).device


def load_model_checkpoint(model: nn.Module, ckpt: str) -> nn.Module:
    """Load model weights from checkpoint file."""
    state_dict = torch.load(ckpt, map_location="cpu")
    if "state_dict" in list(state_dict.keys()):
        state_dict = state_dict["state_dict"]
        try:
            model.load_state_dict(state_dict, strict=False)
        except Exception:
            new_pl_sd = OrderedDict()
            for k, v in state_dict.items():
                new_pl_sd[k] = v
            for k in list(new_pl_sd.keys()):
                if "framestride_embed" in k:
                    new_key = k.replace("framestride_embed", "fps_embedding")
                    new_pl_sd[new_key] = new_pl_sd[k]
                    del new_pl_sd[k]
            model.load_state_dict(new_pl_sd, strict=False)
    else:
        new_pl_sd = OrderedDict()
        for key in state_dict["module"].keys():
            new_pl_sd[key[16:]] = state_dict["module"][key]
        model.load_state_dict(new_pl_sd)

    print(">>> model checkpoint loaded.")
    return model


def write_video(video_path: str, stacked_frames: List[Any], fps: int) -> None:
    """Write a video to disk using imageio."""
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            "pkg_resources is deprecated as an API",
            category=DeprecationWarning,
        )
        imageio.mimsave(video_path, stacked_frames, fps=fps)


def save_results(video: torch.Tensor, filename: str, fps: int = 8) -> None:
    """Save a video tensor as an MP4 file."""
    video = video.detach().cpu()
    video = torch.clamp(video.float(), -1.0, 1.0)
    n = video.shape[0]

    # (B, C, T, H, W) -> (T, B, C, H, W)
    video = video.permute(2, 0, 1, 3, 4)

    frame_grids = [
        torchvision.utils.make_grid(framesheet, nrow=int(n), padding=0)
        for framesheet in video
    ]
    grid = torch.stack(frame_grids, dim=0)  # (T, C, H, W)

    grid = (grid + 1.0) / 2.0
    grid = (grid * 255).to(torch.uint8).permute(0, 2, 3, 1)  # (T, H, W, C)

    torchvision.io.write_video(
        filename,
        grid,
        fps=fps,
        video_codec="h264",
        options={"crf": "10"},
    )


def get_latent_z(model: nn.Module, videos: torch.Tensor) -> torch.Tensor:
    """Encode videos into latent space."""
    b, c, t, h, w = videos.shape
    x = rearrange(videos, "b c t h w -> (b t) c h w")
    z = model.encode_first_stage(x)
    z = rearrange(z, "(b t) c h w -> b c t h w", b=b, t=t)
    return z


def image_guided_synthesis(
    model: torch.nn.Module,
    prompts: list[str],
    observation: Dict[str, torch.Tensor],
    noise_shape: tuple[int, int, int, int, int],
    ddim_steps: int = 50,
    ddim_eta: float = 1.0,
    unconditional_guidance_scale: float = 1.0,
    fs: int | None = None,
    timestep_spacing: str = "uniform",
    guidance_rescale: float = 0.0,
    **kwargs,
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Run inference with DDIM sampling."""
    b, _, t, _, _ = noise_shape

    ddim_sampler = DDIMSampler(model)
    batch_size = noise_shape[0]

    fs = torch.tensor([fs] * batch_size, dtype=torch.long, device=model.device)

    img = observation["observation.images.top"]
    cond_img = img[:, -1, ...]
    cond_img_emb = model.embedder(cond_img)
    cond_img_emb = model.image_proj_model(cond_img_emb)

    if model.model.conditioning_key == "hybrid":
        z = get_latent_z(model, img.permute(0, 2, 1, 3, 4))
        img_cat_cond = z[:, :, -1:, :, :]
        img_cat_cond = repeat(
            img_cat_cond, "b c t h w -> b c (repeat t) h w", repeat=noise_shape[2]
        )
        cond = {"c_concat": [img_cat_cond]}

    cond_ins_emb = model.get_learned_conditioning(prompts)

    cond_state = model.state_projector(observation["observation.state"])
    cond_state_emb = model.agent_state_pos_emb + cond_state

    cond_action = model.action_projector(observation["action"])
    cond_action_emb = model.agent_action_pos_emb + cond_action
    cond_action_emb = torch.zeros_like(cond_action_emb)

    cond["c_crossattn"] = [
        torch.cat([cond_state_emb, cond_ins_emb, cond_img_emb], dim=1)
    ]
    cond["c_crossattn_action"] = [
        observation["observation.images.top"]
        .permute(0, 2, 1, 3, 4)[:, :, -model.n_obs_steps_acting :],
        observation["observation.state"][:, -model.n_obs_steps_acting :],
    ]

    uc = None
    kwargs.update({"unconditional_conditioning_img_nonetext": None})

    cond_mask = None
    cond_z0 = None

    if ddim_sampler is not None:
        samples, actions, states, intermedia = ddim_sampler.sample(
            S=ddim_steps,
            conditioning=cond,
            batch_size=batch_size,
            shape=noise_shape[1:],
            verbose=False,
            unconditional_guidance_scale=unconditional_guidance_scale,
            unconditional_conditioning=uc,
            eta=ddim_eta,
            cfg_img=None,
            mask=cond_mask,
            x0=cond_z0,
            fs=fs,
            timestep_spacing=timestep_spacing,
            guidance_rescale=guidance_rescale,
            **kwargs,
        )

    batch_images = model.decode_first_stage(samples)
    batch_variants = batch_images
    return batch_variants, actions, states


def run_inference(args: argparse.Namespace, gpu_num: int, gpu_no: int) -> Tuple[nn.Module, List[int], Any]:
    """Run inference pipeline on prompts and image inputs."""
    config = OmegaConf.load(args.config)

    # deepspeed backend workaround
    config["model"]["params"]["wma_config"]["params"]["use_checkpoint"] = False

    model = instantiate_from_config(config.model)
    model.perframe_ae = args.perframe_ae

    assert os.path.exists(args.ckpt_path), "Error: checkpoint Not Found!"
    model = load_model_checkpoint(model, args.ckpt_path)
    model = model.cuda(gpu_no)
    model.eval()
    print(">>> Model is successfully loaded ...")

    logging.info("***** Configing Data *****")
    data = instantiate_from_config(config.data)
    data.setup()
    print(">>> Dataset is successfully loaded ...")

    assert (args.height % 16 == 0) and (args.width % 16 == 0), (
        "Error: image size [h,w] should be multiples of 16!"
    )
    assert args.bs == 1, "Current implementation only support [batch size = 1]!"

    h, w = args.height // 8, args.width // 8
    channels = model.model.diffusion_model.out_channels
    n_frames = args.video_length
    print(f">>> Generate {n_frames} frames under each generation ...")
    noise_shape = [args.bs, channels, n_frames, h, w]

    return model, noise_shape, data


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--savedir", type=str, default=None, help="Path to save the results.")
    parser.add_argument("--ckpt_path", type=str, default=None, help="Path to the model checkpoint.")
    parser.add_argument("--config", type=str, help="Path to the config file.")

    parser.add_argument("--ddim_steps", type=int, default=50)
    parser.add_argument("--ddim_eta", type=float, default=1.0)
    parser.add_argument("--bs", type=int, default=1)
    parser.add_argument("--height", type=int, default=320)
    parser.add_argument("--width", type=int, default=512)
    parser.add_argument("--frame_stride", type=int, default=3)
    parser.add_argument("--unconditional_guidance_scale", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--video_length", type=int, default=16)
    parser.add_argument("--timestep_spacing", type=str, default="uniform")
    parser.add_argument("--guidance_rescale", type=float, default=0.0)
    parser.add_argument("--perframe_ae", action="store_true", default=False)
    return parser


class Server:
    def __init__(self, args: argparse.Namespace) -> None:
        self.model_, self.noise_shape_, self.data_ = run_inference(args, 1, 0)
        self.args_ = args
        self.dataset_name = self.data_.dataset_configs["test"]["params"]["dataset_name"]
        self.device_ = get_device_from_parameters(self.model_)

    def normalize_image(self, image: torch.Tensor) -> torch.Tensor:
        return (image / 255.0 - 0.5) * 2.0

    # -------------------- helpers: shape + dim mapping (RAW -> dataset normalizer expected) --------------------
    def _as_2d_td(self, x: torch.Tensor) -> torch.Tensor:
        """Ensure x is 2D and in (T, D) layout."""
        if x.ndim == 0:
            x = x.view(1, 1)
        elif x.ndim == 1:
            # treat as a single timestep
            x = x.unsqueeze(0)  # (1, D)
        elif x.ndim >= 3:
            # collapse leading dims except last one, keeping time as first dim if possible
            x = x.reshape(x.shape[0], -1)
        return x

    def _infer_norm_feature_dim(self, key: str) -> Optional[int]:
        """Try to read normalizer stats dim for `key` (e.g. 'observation.state' or 'action')."""
        norm = getattr(self.data_.test_datasets[self.dataset_name], "normalizer", None)
        if norm is None:
            return None

        # common patterns in Normalize modules
        for attr in ("min", "mins", "_min", "_mins", "minimum", "minimums"):
            if hasattr(norm, attr):
                v = getattr(norm, attr)
                if isinstance(v, dict) and key in v and isinstance(v[key], torch.Tensor):
                    return int(v[key].numel())

        for attr in ("max", "maxs", "_max", "_maxs", "maximum", "maximums"):
            if hasattr(norm, attr):
                v = getattr(norm, attr)
                if isinstance(v, dict) and key in v and isinstance(v[key], torch.Tensor):
                    return int(v[key].numel())

        # fallback: try internal dict of stats
        for attr in ("stats", "_stats", "data_stats", "_data_stats"):
            if hasattr(norm, attr):
                v = getattr(norm, attr)
                if isinstance(v, dict) and key in v and isinstance(v[key], dict):
                    # maybe {'min': tensor, 'max': tensor}
                    d = v[key]
                    for kk in ("min", "max"):
                        if kk in d and isinstance(d[kk], torch.Tensor):
                            return int(d[kk].numel())

        return None

    def _select_7_from_14_16(self, x: torch.Tensor, want: int) -> torch.Tensor:
        """
        When input is dual (14/16) but dataset wants 7:
          - default choose left [0:7]
          - right choose [7:14]
        """
        arm = os.getenv("UNIFOLM_ARM_SELECT", "left").strip().lower()
        if want != 7:
            return x[:, :want]

        if x.shape[1] >= 14:
            if arm == "right":
                return x[:, 7:14]
            return x[:, 0:7]
        return x[:, :7]

    def _fit_feature_dim(self, x: torch.Tensor, want: int, key_name: str) -> torch.Tensor:
        """Crop/pad feature dim to `want` (operate on last dim)."""
        x = x.float()
        have = x.shape[1]
        if have == want:
            return x

        # special case: dual->single arm
        if want == 7 and have in (14, 16):
            return self._select_7_from_14_16(x, want)

        # generic crop
        if have > want:
            return x[:, :want]

        # generic pad
        pad = torch.zeros((x.shape[0], want - have), dtype=x.dtype, device=x.device)
        return torch.cat([x, pad], dim=1)

    def _maybe_deg_to_rad(self, x: torch.Tensor) -> torch.Tensor:
        """Optionally convert degrees->radians if user forces it."""
        unit = os.getenv("UNIFOLM_INPUT_UNIT", "").strip().lower()
        if unit in ("deg", "degree", "degrees"):
            return x * (torch.pi / 180.0)
        return x

    def _maybe_rad_to_deg(self, x: torch.Tensor) -> torch.Tensor:
        """Optionally convert radians->degrees for output action."""
        unit = os.getenv("UNIFOLM_ACTION_OUTPUT_UNIT", "").strip().lower()
        if unit in ("deg", "degree", "degrees"):
            return x * (180.0 / torch.pi)
        return x
    # ----------------------------------------------------------------------------------------------------------

    def predict_action(self, payload: Dict[str, Any]) -> Any:
        try:
            images = payload["observation.images.top"]
            states = payload["observation.state"]
            actions = payload["action"]  # Should be all zeros
            language_instruction = payload["language_instruction"]

            # ---------- images ----------
            images = torch.tensor(images).cuda()
            images = self.data_.test_datasets[self.dataset_name].spatial_transform(images).unsqueeze(0)
            images = self.normalize_image(images)
            print(f"images shape: {images.shape} ...")

            # ---------- states (RAW) ----------
            states = torch.tensor(states)
            states = self._as_2d_td(states)  # (T, D?) or (D, T?) still possible

            # Heuristic transpose: pick orientation where feature dim looks like 6/7/14/16
            # If states is (D, T) and D looks like feature dim while T looks like time (2/16), transpose.
            if states.ndim == 2:
                T, D = states.shape[0], states.shape[1]
                # if first dim looks like feature dim and second dim looks like time
                if T in (6, 7, 14, 16) and D not in (6, 7, 14, 16):
                    states = states.t().contiguous()

            # convert unit if forced
            states = self._maybe_deg_to_rad(states)

            print(f"raw states shape(after ensure): {states.shape} ...")

            # IMPORTANT: normalizer for this dataset expects RAW dim (e.g. z1_stackbox expects 7)
            want_state_dim = self._infer_norm_feature_dim("observation.state")
            if want_state_dim is None:
                # safe fallback: z1 often 7 (6 joints + gripper/placeholder), otherwise keep as-is
                want_state_dim = 7 if "z1" in str(self.dataset_name).lower() else states.shape[1]

            states = self._fit_feature_dim(states, want_state_dim, "observation.state")

            states = self.data_.test_datasets[self.dataset_name].normalizer(
                {"observation.state": states}
            )["observation.state"]

            # map RAW -> unified state for model
            states, _ = self.data_.test_datasets[self.dataset_name]._map_to_uni_state(states, "joint position")
            print(f"states shape(after map): {states.shape} ...")

            # ---------- actions (RAW) ----------
            actions = torch.tensor(actions)
            actions = self._as_2d_td(actions)

            if actions.ndim == 2:
                T, D = actions.shape[0], actions.shape[1]
                if T in (6, 7, 14, 16) and D not in (6, 7, 14, 16):
                    actions = actions.t().contiguous()

            actions = self._maybe_deg_to_rad(actions)
            print(f"raw actions shape(after ensure): {actions.shape} ...")

            want_action_dim = self._infer_norm_feature_dim("action")
            if want_action_dim is None:
                want_action_dim = 7 if "z1" in str(self.dataset_name).lower() else actions.shape[1]

            actions = self._fit_feature_dim(actions, want_action_dim, "action")

            # NOTE: some codebases normalize action inside _map_to_uni_action, but yours expects RAW->map,
            # so keep the original order you had: map then later unnormalize uses dataset unnormalizer.
            actions, action_mask = self.data_.test_datasets[self.dataset_name]._map_to_uni_action(
                actions, "joint position"
            )
            print(f"actions shape(after map): {actions.shape} ...")
            print("=" * 20)

            # batchify
            states = states.unsqueeze(0).cuda()
            actions = actions.unsqueeze(0).cuda()

            observation = {
                "observation.images.top": images,
                "observation.state": states,
                "action": actions,
            }
            observation = {k: v.to(self.device_, non_blocking=True) for k, v in observation.items()}

            args = self.args_
            pred_videos, pred_action, _ = image_guided_synthesis(
                self.model_,
                language_instruction,
                observation,
                self.noise_shape_,
                ddim_steps=args.ddim_steps,
                ddim_eta=args.ddim_eta,
                unconditional_guidance_scale=args.unconditional_guidance_scale,
                fs=30 / args.frame_stride,
                timestep_spacing=args.timestep_spacing,
                guidance_rescale=args.guidance_rescale,
            )

            # select valid dims by mask and unnormalize back to RAW action space
            pred_action = pred_action[..., action_mask[0] == 1.0][0].cpu()
            pred_action = self.data_.test_datasets[self.dataset_name].unnormalizer({"action": pred_action})["action"]

            # optional unit convert for downstream robot (e.g., D1 expects degrees)
            if isinstance(pred_action, torch.Tensor):
                pred_action = self._maybe_rad_to_deg(pred_action)

            os.makedirs(args.savedir, exist_ok=True)
            current_time = datetime.now().strftime("%H:%M:%S")
            video_file = f"{args.savedir}/{current_time}.mp4"
            save_results(pred_videos.cpu(), video_file)

            response = {"result": "ok", "action": pred_action.tolist(), "desc": "success"}
            return JSONResponse(response)

        except Exception:
            logging.error(traceback.format_exc())
            logging.warning(
                "Your request threw an error; make sure your request complies with the expected format:\n"
                "{'observation.images.top': ..., 'observation.state': ..., 'action': ..., 'language_instruction': ...}\n"
            )
            return {"result": "error", "desc": traceback.format_exc()}

    def run(self, host: str | None = None, port: int | None = None) -> None:
        if host is None:
            host = os.environ.get("UNIFOLM_SERVER_HOST", "127.0.0.1")
        if port is None:
            port = int(os.environ.get("UNIFOLM_SERVER_PORT", "8000"))

        self.app = FastAPI()
        self.app.post("/predict_action")(self.predict_action)

        print(">>> Inference server is ready ... ")
        uvicorn.run(self.app, host=host, port=port)
        print(">>> Inference server stops ... ")
        return


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()
    seed_everything(args.seed)

    print(">>> Launch inference server ... ")
    server = Server(args)
    server.run()
