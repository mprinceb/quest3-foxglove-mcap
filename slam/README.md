# SLAM Implementation Bootstrap

This folder contains the first runnable SLAM ingestion pipeline for Quest 3 sessions.

## Single Command Pipeline (Data -> EuRoC -> MCAP)
This is now the default entrypoint and follows the conversion flow described in Foxglove's EuRoC article:
https://foxglove.dev/blog/converting-euroc-mav-dataset-to-mcap

```bash
bash slam/runners/run_data_to_euroc_to_mcap.sh data/20260209_173413 --include-depth
```

What it does:
1. validates the capture;
2. exports EuRoC layout under `slam/results/pipeline_<session>/euroc`;
3. runs baseline depth+PnP VO (unless disabled);
4. converts EuRoC output to ROS2-profile MCAP for Foxglove.

Final artifact:
- `slam/results/pipeline_<session>/<session>_slam.mcap`

Dependency note:
- ROS2-profile MCAP writing uses `mcap-ros2-support` (declared in `pyproject.toml`).

## 1) Validate a Capture
```bash
uv run python slam/qa/check_capture.py --session-dir data/20260209_173413
```

## 2) Export to EuRoC-style Layout (Stereo + IMU)
```bash
uv run python slam/adapters/export_euroc.py \
  --session-dir data/20260209_173413 \
  --out-dir slam/results/euroc_20260209_173413 \
  --include-depth
```

## 3) Export Reference Trajectories (TUM)
```bash
uv run python slam/adapters/export_tum.py \
  --session-dir data/20260209_173413 \
  --out-dir slam/results/euroc_20260209_173413/reference
```

## 4) Run ORB-SLAM3 Stereo
Prerequisites:
- ORB-SLAM3 built locally
- `ORB_SLAM3_ROOT` set, or explicit `ORB_SLAM3_STEREO_BIN` and `ORB_VOCAB`

```bash
bash slam/runners/run_orbslam3_stereo.sh \
  slam/results/euroc_20260209_173413
```

## 5) Run In-Repo Baseline VO (No External SLAM Dependency)
This provides an immediate trajectory baseline from left camera + depth:
```bash
bash slam/runners/run_depth_pnp_vo.sh data/20260209_173413
```
Output goes to `slam/results/depth_pnp_vo_<session>.tum.txt`.

## Notes
- `export_euroc.py` exports grayscale PNG images from Y plane for fast stereo ingestion.
- `cam1` translation in `sensor.yaml` is initialized from median depth-descriptor offset. Recalibrate before production use.
- `imu0/sensor.yaml` noise values are placeholders and should be replaced after Allan variance/calibration.
- `euroc_to_mcap.py` writes ROS2 message schemas (`sensor_msgs/Image`, `sensor_msgs/Imu`, `sensor_msgs/CameraInfo`, `geometry_msgs/PoseStamped`) so the MCAP is visualization-ready and closer to RTAB-map needs.
