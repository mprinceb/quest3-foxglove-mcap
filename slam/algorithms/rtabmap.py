"""RTAB-Map playback helper using ROS2 CLI."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


def run_rtabmap_from_mcap(mcap_path: Path) -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError("ros2 not found in PATH. Install/source ROS2 first.")

    mcap_path = mcap_path.expanduser().resolve()
    if not mcap_path.exists():
        raise FileNotFoundError(f"MCAP file not found: {mcap_path}")

    bag_proc = subprocess.Popen(["ros2", "bag", "play", str(mcap_path)])
    try:
        subprocess.run(
            [
                "ros2",
                "launch",
                "rtabmap_launch",
                "rtabmap.launch.py",
                "stereo:=true",
                "visual_odometry:=true",
                "approx_sync:=true",
                "left_image_topic:=/cam0/image_raw",
                "right_image_topic:=/cam1/image_raw",
                "left_camera_info_topic:=/cam0/camera_info",
                "right_camera_info_topic:=/cam1/camera_info",
                "imu_topic:=/imu0",
            ],
            check=True,
        )
    finally:
        bag_proc.terminate()


def add_parser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser = subparsers.add_parser("run-rtabmap", help="Play MCAP and launch RTAB-Map with topic remaps")
    parser.add_argument("--mcap", required=True)
    return parser


def run_cli(args: argparse.Namespace) -> int:
    run_rtabmap_from_mcap(Path(args.mcap))
    return 0
