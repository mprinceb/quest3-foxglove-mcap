#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <slam_mcap_path>"
  echo "Example: $0 slam/results/pipeline_20260209_173413_full/20260209_173413_slam.mcap"
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Install/source ROS2 first."
  exit 2
fi

BAG_PATH="$(realpath "$1")"

# Verify available args in your local rtabmap_launch package:
#   ros2 launch rtabmap_launch rtabmap.launch.py --show-args

ros2 bag play "$BAG_PATH" &
BAG_PID=$!
trap 'kill $BAG_PID >/dev/null 2>&1 || true' EXIT

sleep 2

ros2 launch rtabmap_launch rtabmap.launch.py \
  stereo:=true \
  visual_odometry:=true \
  approx_sync:=true \
  left_image_topic:=/cam0/image_raw \
  right_image_topic:=/cam1/image_raw \
  left_camera_info_topic:=/cam0/camera_info \
  right_camera_info_topic:=/cam1/camera_info \
  imu_topic:=/imu0
