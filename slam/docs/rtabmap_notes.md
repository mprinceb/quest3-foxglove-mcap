# RTAB-Map Next-Step Notes

## Current Status
`quest-slam pipeline` generates a ROS2-profile MCAP with:
- `/cam0/image_raw`
- `/cam1/image_raw`
- `/cam0/camera_info`, `/cam1/camera_info`
- `/imu0`
- optional `/depth0/image_raw`, `/depth1/image_raw`
- `/groundtruth/pose`, `/slam/estimated_pose`

This is the correct intermediate format for Foxglove and ROS2 playback.

## Recommended Bringup Flow
1. Generate MCAP:
```bash
uv run python -m slam.cli pipeline --session-dir data/<session> --include-depth --image-mode mono
```

2. Inspect local RTAB-map launch args:
```bash
ros2 launch rtabmap_launch rtabmap.launch.py --show-args
```

3. Launch with built-in helper:
```bash
uv run python -m slam.cli run-rtabmap --mcap slam/results/pipeline_<session>/<session>_slam.mcap
```

## Topic Mapping Used
- left image: `/cam0/image_raw`
- right image: `/cam1/image_raw`
- left camera info: `/cam0/camera_info`
- right camera info: `/cam1/camera_info`
- imu: `/imu0`

## Important Caveat
`/cam1/camera_info` projection baseline currently uses metadata-derived baseline magnitude.
For production RTAB-map runs, replace with calibrated stereo extrinsics.

## References
- Foxglove EuRoC conversion tutorial:
  https://foxglove.dev/blog/converting-euroc-mav-dataset-to-mcap
- RTAB-Map ROS2:
  https://github.com/introlab/rtabmap_ros
