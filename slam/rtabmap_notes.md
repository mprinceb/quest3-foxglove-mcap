# RTAB-Map Next-Step Notes

## Current Status
`slam/pipeline_data_to_mcap.py` now generates a ROS2-profile MCAP with:
- `/cam0/image_raw` (`sensor_msgs/msg/Image`, `mono8`)
- `/cam1/image_raw` (`sensor_msgs/msg/Image`, `mono8`)
- `/cam0/camera_info`, `/cam1/camera_info`
- `/imu0` (`sensor_msgs/msg/Imu`)
- `/depth0/image_raw`, `/depth1/image_raw` (optional)
- `/groundtruth/pose` and `/slam/estimated_pose` (`geometry_msgs/msg/PoseStamped`)

This is the right intermediate format for Foxglove visualization and for wiring RTAB-map with topic remaps.

## Recommended RTAB-Map Bringup Flow
1. Play MCAP with ROS2:
```bash
ros2 bag play slam/results/pipeline_<session>/<session>_slam.mcap
```

2. Inspect `rtabmap_launch` arguments in your installed version:
```bash
ros2 launch rtabmap_launch rtabmap.launch.py --show-args
```

3. Start stereo odometry + mapping with explicit topic remaps/args.
Use your local `--show-args` output to set:
- left image topic -> `/cam0/image_raw`
- right image topic -> `/cam1/image_raw`
- left camera info -> `/cam0/camera_info`
- right camera info -> `/cam1/camera_info`
- imu topic -> `/imu0`

Convenience runner in this repo:
```bash
bash slam/runners/run_rtabmap_from_mcap.sh \
  slam/results/pipeline_<session>/<session>_slam.mcap
```

## Important Calibration Caveat
`/cam1/camera_info` projection baseline currently uses a heuristic baseline magnitude inferred from exported metadata.
Before production RTAB-map evaluation, replace this with calibrated stereo extrinsics from a proper calibration run.

## References
- Foxglove EuRoC conversion tutorial:
  https://foxglove.dev/blog/converting-euroc-mav-dataset-to-mcap
- RTAB-Map ROS2 launch package:
  https://github.com/introlab/rtabmap_ros
