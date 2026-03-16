import json
import os
import shutil
import h5py
import argparse
import pandas as pd
import torch
import subprocess
import numpy as np

from pathlib import Path
from safetensors.torch import save_file
from tqdm import tqdm


def flatten_dict(d, parent_key="", sep="/"):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def is_av1(file_path):
    try:
        result = subprocess.run([
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=codec_name", "-of", "csv=p=0",
            str(file_path)
        ], capture_output=True, text=True, check=True)
        return result.stdout.strip() == "av1"
    except subprocess.CalledProcessError:
        return False


def convert_to_h264(input_path, output_path):
    subprocess.run([
        "ffmpeg", "-y", "-i", str(input_path), "-c:v", "libx264", "-preset", "slow", "-crf", "23",
        "-c:a", "copy", str(output_path)
    ], check=True)


def maybe_copy_or_convert_video(source_video: Path, target_video: Path):
    if not source_video.exists():
        raise FileNotFoundError(f"Missing source video: {source_video}")
    if is_av1(source_video):
        print(f"Converting {source_video.name} to H.264...")
        convert_to_h264(source_video, target_video)
    else:
        shutil.copy2(source_video, target_video)


def maybe_deg_to_rad(x: torch.Tensor, enabled: bool) -> torch.Tensor:
    if not enabled:
        return x
    return torch.deg2rad(x)


def main(args):
    source_dir = Path(args.source_dir)
    source_data_dir = source_dir / args.dataset_name / "data" / "chunk-000"
    source_meta_dir = source_dir / args.dataset_name / "meta"
    source_videos_dir = source_dir / args.dataset_name / "videos" / "chunk-000"

    target_dir = Path(args.target_dir)
    target_videos_dir = target_dir / "videos" / args.dataset_name
    target_transitions_dir = target_dir / "transitions" / args.dataset_name
    target_meta_dir = target_transitions_dir / "meta_data"

    target_dir.mkdir(parents=True, exist_ok=True)
    target_videos_dir.mkdir(parents=True, exist_ok=True)
    target_transitions_dir.mkdir(parents=True, exist_ok=True)
    target_meta_dir.mkdir(parents=True, exist_ok=True)

    csv_file = target_dir / f"{args.dataset_name}.csv"
    COLUMNS = [
        'videoid', 'contentUrl', 'duration', 'data_dir', 'instruction',
        'dynamic_confidence', 'dynamic_wording', 'dynamic_source_category',
        'embodiment'
    ]
    df = pd.DataFrame(columns=COLUMNS)

    info_json_path = source_meta_dir / "info.json"
    with open(str(info_json_path), "r") as f:
        info = json.load(f)
    total_episodes = info['total_episodes']

    tasks_jsonl_path = source_meta_dir / "tasks.jsonl"
    with open(str(tasks_jsonl_path), "r") as f:
        tasks = [json.loads(line) for line in f]
    instruction = tasks[0]['task']

    source_video_views = sorted([d for d in source_videos_dir.iterdir() if d.is_dir()])
    all_actions = []
    all_states = []

    for v_idx, source_view_dir in enumerate(source_video_views):
        view_name = source_view_dir.name
        target_videos_view_dir = target_videos_dir / view_name
        target_videos_view_dir.mkdir(parents=True, exist_ok=True)

        for idx in tqdm(range(total_episodes), desc=f"processing {view_name}"):
            source_video = source_view_dir / f"episode_{idx:06d}.mp4"
            target_video = target_videos_view_dir / f"{idx}.mp4"
            maybe_copy_or_convert_video(source_video, target_video)

            episode_parquet_file = source_data_dir / f"episode_{idx:06d}.parquet"
            episode_data = pd.read_parquet(episode_parquet_file)
            actions = torch.tensor(episode_data['action'].tolist(), dtype=torch.float32)
            states = torch.tensor(episode_data['observation.state'].tolist(), dtype=torch.float32)

            actions = maybe_deg_to_rad(actions, args.convert_deg_to_rad)
            states = maybe_deg_to_rad(states, args.convert_deg_to_rad)

            if v_idx == 0:
                target_h5_file = target_transitions_dir / f"{idx}.h5"
                with h5py.File(str(target_h5_file), 'w') as h5f:
                    h5f.create_dataset('observation.state', data=states.numpy())
                    h5f.create_dataset('action', data=actions.numpy())
                    h5f.attrs['action_type'] = 'joint position'
                    h5f.attrs['state_type'] = 'joint position'
                    h5f.attrs['robot_type'] = args.robot_name
                all_actions.append(actions)
                all_states.append(states)

            row = {
                'videoid': idx,
                'contentUrl': 'x',
                'duration': 'x',
                'data_dir': args.dataset_name + f"/{view_name}",
                'instruction': instruction,
                'dynamic_confidence': 'x',
                'dynamic_wording': 'x',
                'dynamic_source_category': 'x',
                'embodiment': args.robot_name
            }
            df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)

    actions = torch.cat(all_actions, dim=0)
    states = torch.cat(all_states, dim=0)

    stats = {'action': {}, 'observation.state': {}}
    stats['action']['max'] = actions.max(dim=0).values
    stats['action']['min'] = actions.min(dim=0).values
    stats['action']['mean'] = actions.mean(dim=0)
    stats['action']['std'] = actions.std(dim=0)

    stats['observation.state']['max'] = states.max(dim=0).values
    stats['observation.state']['min'] = states.min(dim=0).values
    stats['observation.state']['mean'] = states.mean(dim=0)
    stats['observation.state']['std'] = states.std(dim=0)

    flattened_stats = flatten_dict(stats)
    target_stats_file = target_meta_dir / "stats.safetensors"
    save_file(flattened_stats, target_stats_file)

    df.to_csv(csv_file, index=False)
    print(f">>> Finished create {args.dataset_name} dataset ...")
    if args.convert_deg_to_rad:
        print(">>> NOTE: observation.state and action were converted from degree to radian.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--source_dir', action='store', type=str, required=True,
                        help='Root directory containing LeRobot-style dataset folders.')
    parser.add_argument('--target_dir', action='store', type=str, default='./data',
                        help='The target dir to save new formatted dataset.')
    parser.add_argument('--dataset_name', action='store', type=str, required=True,
                        help='dataset name')
    parser.add_argument('--robot_name', action='store', type=str, required=True,
                        help='robot name')
    parser.add_argument('--convert_deg_to_rad', action='store_true',
                        help='Convert observation.state and action from degree to radian before saving.')
    main(parser.parse_args())
