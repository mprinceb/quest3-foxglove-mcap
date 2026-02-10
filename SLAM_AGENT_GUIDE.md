# SLAM Agent Development Guide (Quest 3 -> MCAP -> SLAM)

## 1) Goal and Current State
We already generate MCAP from Quest 3 captures (stereo frames + head pose, and controller poses available).  
Next goal: add a robust SLAM pipeline with measurable quality (trajectory accuracy, drift, relocalization) and reproducible outputs.

This guide is the execution contract for agents working on SLAM in this repository.

## 2) Why This Plan
SLAM failures are usually data/calibration/timestamp failures, not algorithm choice alone.  
So we implement in this order:
1. Data contract and timing discipline
2. Baseline stereo SLAM from existing data
3. Evaluation harness
4. IMU capture + calibration
5. Stereo-inertial SLAM selection for production

This de-risks quickly while keeping future algorithm options open.

## 3) Research-Backed Candidate Stack
Shortlist from web/GitHub research:
- ORB-SLAM3 (stereo/mono/RGB-D + inertial; strong baseline; GPL-3.0)
- VINS-Fusion (stereo and stereo-inertial; loop closure; GPL-3.0)
- OpenVINS (mature VIO estimation and calibration flow; GPL-3.0)
- Basalt (VIO/mapping + calibration; permissive BSD-3-Clause)
- Kimera-VIO (stereo+IMU, BSD-2-Clause)
- Isaac ROS Visual SLAM / cuVSLAM (ROS2 + GPU-oriented; Apache-2.0)

Working strategy:
- Start now with ORB-SLAM3 stereo on current logs (fastest signal)
- Move to stereo+IMU evaluation with Basalt/OpenVINS once IMU logging is added
- Choose production stack by metrics + runtime + license constraints

## 4) File-Based Work Breakdown for Agents
Use these repository additions and keep ownership boundaries strict.

1. `slam/data_contract.md`
- Define required topics, frames, units, and timestamp rules.
- Include topic table for `/camera/left/*`, `/camera/right/*`, `/tf`, `/imu/head`, `/imu/left_controller`, `/imu/right_controller`.

2. `slam/adapters/export_euroc.py`
- Convert MCAP sessions to EuRoC-style folders for offline runners.
- Output image timestamps CSV and IMU CSV with nanosecond alignment.

3. `slam/adapters/export_tum.py`
- Export trajectory for APE/RPE comparison with `evo`.

4. `slam/runners/run_orbslam3_stereo.sh`
- Repro script for ORB-SLAM3 stereo baseline.

5. `slam/runners/run_openvins.sh` and `slam/runners/run_basalt.sh`
- Stereo-inertial runs once IMU is available.

6. `slam/eval/evaluate.sh`
- Run `evo_ape` / `evo_rpe`, save plots and summary metrics.

7. `slam/results/<algo>/<session>/`
- Store trajectory outputs, metrics JSON/TXT, and logs.

## 5) Acceptance Criteria by Phase
Phase 1 (Data contract):
- Same session replay is deterministic.
- Timestamp monotonicity checks pass.

Phase 2 (Stereo baseline):
- ORB-SLAM3 produces continuous trajectory on at least one real session.

Phase 3 (Evaluation):
- APE/RPE report is generated for each run.
- Regression threshold documented (for example, fail if APE RMSE worsens >10%).

Phase 4 (IMU integration):
- IMU is logged at stable high rate (target: >= 100 Hz).
- Camera-IMU temporal offset estimated and documented.

Phase 5 (Production selection):
- Decision memo compares at least 2 stacks on quality + runtime + integration cost.

## 6) What IMU Means on Quest 3
IMU = inertial measurement unit, mainly:
- Gyroscope: angular velocity (rad/s)
- Accelerometer: linear acceleration (m/s^2)

For SLAM/VIO, IMU improves robustness during fast motion, motion blur, and low-texture scenes.

Important: controller pose streams are useful, but they are not a replacement for head-mounted IMU in visual-inertial SLAM.

## 7) How to Get IMU Data from Quest 3
### Practical path (recommended)
Log motion features from the XR runtime in your capture app for:
- `XRNode.Head`
- `XRNode.LeftHand`
- `XRNode.RightHand`

In Unity XR APIs, query features such as:
- `deviceAngularVelocity`
- `deviceVelocity`
- `deviceAcceleration` (if provider/runtime exposes it)
- `deviceAngularAcceleration` (if exposed)

If a feature is unavailable on a runtime, record validity flags and do not fake values.

### OpenXR note
Core OpenXR provides pose and space velocity; acceleration is not guaranteed in core space-velocity structures.  
If you need acceleration but it is unavailable, use provider-specific APIs or derive cautiously from velocity differences (with filtering and explicit quality flags).

### Logging rules
1. Use monotonic timestamps (ns) from one clock domain.
2. Log per sample: timestamp, frame_id, sensor_id, values, validity.
3. Persist to MCAP and mirror to CSV for quick debugging.
4. Write IMU into ROS-compatible shape (`sensor_msgs/Imu` fields) even if consumed offline first.

### Minimal schema recommendation (`/imu/head`)
- `timestamp_ns`
- `angular_velocity_xyz` (rad/s)
- `linear_acceleration_xyz` (m/s^2)
- `orientation_xyzw` (optional; if coming from tracker fusion)
- covariance fields and validity bits

## 8) Immediate Next Sprint (Execution Order)
1. Keep `slam/pipeline_data_to_mcap.py` as the canonical single-command flow:
   data session -> EuRoC export -> MCAP output.
2. Ensure `slam/adapters/euroc_to_mcap.py` remains compatible with Foxglove visualization.
3. Run ORB-SLAM3 stereo baseline and export estimated trajectory into the same MCAP.
4. Add `evo` evaluation script and publish baseline report.
5. Add stereo-inertial run path (Basalt/OpenVINS), then compare.
6. Introduce RTAB-map-compatible topic mapping and launch configs.

## 9) Sources Used
- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
- VINS-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- OpenVINS docs: https://docs.openvins.com/
- OpenVINS repo: https://github.com/rpng/open_vins
- Basalt: https://gitlab.com/VladyslavUsenko/basalt
- Kimera-VIO: https://github.com/MIT-SPARK/Kimera-VIO
- Isaac ROS Visual SLAM: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
- ROS2 rosbag2 (MCAP default): https://github.com/ros2/rosbag2
- MCAP ROS2 guide: https://mcap.dev/guides/getting-started/ros-2
- OpenXR `XrSpaceVelocity`: https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#XrSpaceVelocity
- Unity XR CommonUsages (velocity/acceleration features): https://docs.unity.cn/cn/tuanjiemanual/1.6/ScriptReference/XR.CommonUsages.html
- Unity XRNodeState (velocity/acceleration accessors): https://docs.unity3d.com/ScriptReference/XR.XRNodeState.html
- Foxglove EuRoC conversion tutorial: https://foxglove.dev/blog/converting-euroc-mav-dataset-to-mcap
- RTAB-Map ROS2: https://github.com/introlab/rtabmap_ros
