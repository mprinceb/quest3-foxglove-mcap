# VIO Agent Development Guide (Quest 3 -> MCAP -> VIO)

## 1) Goal and Current State
We already generate MCAP from Quest 3 captures with stereo camera streams and head pose transforms.
Next goal: implement a robust visual-inertial odometry (VIO) pipeline that is measurable, reproducible, and production-selectable.

This guide is the execution contract for agents working on VIO in this repository.

## 2) Why This Plan
Most VIO failures come from timestamp discipline, sensor calibration, and frame conventions, not estimator choice alone.
So we execute in this order:
1. VIO data contract and clock discipline
2. Stereo baseline from existing logs (no IMU)
3. Evaluation harness and regression thresholds
4. IMU capture integration (head first)
5. Camera-IMU calibration (spatial + temporal)
6. Stereo-inertial VIO comparison and production selection

This sequence de-risks quickly and keeps algorithm options open.

## 3) Candidate Estimator Stack
Shortlist from research and ecosystem maturity:
- OpenVINS (stereo-inertial MSCKF, active project, strong research tooling, GPL-3.0)
- Basalt (VIO + calibration tooling, BSD-3-Clause)
- ORB-SLAM3 stereo-inertial mode (strong baseline and loop-closure path, GPL-3.0)
- VINS-Fusion (stereo/stereo-inertial + loop closure, GPL-3.0)
- Kimera-VIO (stereo+IMU, BSD-2-Clause)
- Isaac ROS Visual SLAM (stereo VIO/SLAM, ROS2 + NVIDIA stack, Apache-2.0)

Working strategy:
- Use ORB-SLAM3 stereo now for immediate baseline signal on existing data.
- Add head IMU + calibration, then benchmark OpenVINS vs Basalt for VIO.
- Choose production stack by quality, runtime, integration friction, and license constraints.

## 4) VIO Data Contract (Mandatory)
Use a strict, versioned contract before adding estimator code.

1. Required topics
- `/camera/left/image` and `/camera/right/image` (or canonical equivalents)
- `/camera/left/calib` and `/camera/right/calib`
- `/imu/head` (required for VIO)
- `/tf` with world->head transform (optional as reference/diagnostic)

2. Required units
- Timestamp: nanoseconds in one monotonic clock domain
- Gyro: rad/s
- Accel: m/s^2
- Position: meters
- Quaternion: xyzw, normalized

3. Required metadata
- `frame_id` and sensor IDs per message
- Validity flags for each IMU field if runtime/provider may omit values
- Covariance fields (explicitly set even if approximate)

4. Clock rules
- No mixed clock domains inside one session
- Monotonic increase within each topic
- Camera and IMU time offset is measured/documented, not assumed zero

## 5) Repository Work Breakdown
Add these files and keep ownership boundaries strict.

1. `vio/data_contract.md`
- VIO-specific topic table, frame definitions, unit rules, and timestamp policy.

2. `vio/qa/check_timestamps.py`
- Validate monotonicity, jitter, dropped-frame patterns, and camera/IMU alignment diagnostics.

3. `vio/adapters/export_euroc.py`
- Convert MCAP sessions into EuRoC-style directory structure for offline VIO runners.

4. `vio/adapters/export_tum.py`
- Export trajectories in TUM format for metric evaluation (`evo_ape`, `evo_rpe`).

5. `vio/runners/run_orbslam3_stereo.sh`
- Stereo baseline runner (no IMU) for immediate signal.

6. `vio/runners/run_openvins.sh`
- Stereo-inertial VIO run script using calibrated camera/IMU settings.

7. `vio/runners/run_basalt.sh`
- Stereo-inertial VIO run script and logs.

8. `vio/eval/evaluate.sh`
- Generate APE/RPE metrics, summaries, and comparison plots.

9. `vio/results/<algo>/<session>/`
- Store trajectories, config snapshots, logs, metrics, and run metadata.

## 6) Acceptance Criteria by Phase
Phase 1 (Contract + QA)
- Timestamp monotonicity checks pass.
- Sensor units and frame conventions validated on a real session.

Phase 2 (Stereo baseline)
- Continuous trajectory on at least one real capture.
- Baseline metrics artifact generated.

Phase 3 (IMU integration)
- Head IMU rate stable (target: >= 100 Hz)
- No silent missing IMU fields; validity bits present.

Phase 4 (Calibration)
- Camera intrinsics/extrinsics and camera-IMU time offset estimated and versioned.
- Calibration residuals/report committed under results or docs.

Phase 5 (VIO comparison)
- At least two stereo-inertial stacks compared on identical sessions.
- Decision memo includes quality + runtime + integration + license summary.

## 7) Quest 3 IMU Acquisition Guidance
Practical path:
- Capture head motion features from XR runtime (`XRNode.Head`) and persist as `/imu/head`.
- Also log controller IMU-like streams only as optional diagnostics, not primary VIO input.

Unity/XR notes:
- Query motion features such as angular velocity and acceleration where available.
- If provider/runtime omits a field, set validity false; do not fabricate values.

OpenXR note:
- Core OpenXR guarantees velocity query path via `XrSpaceVelocity` semantics.
- Acceleration is not guaranteed in core APIs; use provider-specific access when available.

Logging rules:
1. Use one monotonic timestamp source in ns.
2. Log per-sample validity bits for each vector.
3. Preserve raw values without smoothing in primary logs.
4. Keep optional filtered streams as separate topics if needed.

## 8) Calibration Strategy
1. Camera-only calibration
- Validate intrinsics and stereo extrinsics from capture metadata; recalibrate if drift/suspect.

2. Camera-IMU calibration
- Estimate spatial extrinsics and temporal offset with a calibration toolchain (Kalibr/Basalt flows).
- Repeat with multiple motion patterns (slow/fast rotations and translations).

3. Re-calibration triggers
- Hardware mount change
- Runtime/firmware update affecting sensor timing
- Significant metric regressions without code changes

## 9) Evaluation Protocol
For every VIO run:
1. Export reference trajectory and estimator trajectory in comparable format.
2. Compute APE and RPE (`evo_ape`, `evo_rpe`).
3. Record tracking dropouts, runtime stats, and failure windows.
4. Store run config hash + calibration version with metrics.

Minimum report fields:
- Session ID
- Estimator + commit/tag
- Calibration version
- APE RMSE / median
- RPE translational and rotational error
- Runtime (avg and peak), hardware target
- Notable failure cases

## 10) Production Selection Rules
Pick production estimator using weighted criteria:
- 40% trajectory quality and robustness
- 25% runtime and resource usage on target hardware
- 20% integration complexity and maintainability
- 15% license/compliance fit for deployment

Keep one fallback estimator runnable via script for regressions and incident triage.

## 11) Immediate Next Sprint (Execution Order)
1. Implement `vio/data_contract.md`.
2. Add `/imu/head` writing path in capture->MCAP pipeline.
3. Add timestamp QA checker (`vio/qa/check_timestamps.py`).
4. Build `vio/adapters/export_euroc.py`.
5. Run ORB-SLAM3 stereo baseline and publish first metrics.
6. Add OpenVINS and Basalt runners for stereo-inertial comparison.

## 12) Sources
- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
- OpenVINS docs: https://docs.openvins.com/
- OpenVINS repo: https://github.com/rpng/open_vins
- VINS-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- Basalt: https://github.com/VladyslavUsenko/basalt
- Kimera-VIO: https://github.com/MIT-SPARK/Kimera-VIO
- Isaac ROS Visual SLAM: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
- Kalibr: https://github.com/ethz-asl/kalibr
- evo: https://github.com/MichaelGrupp/evo
- OpenXR spec (`XrSpaceVelocity`): https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#XrSpaceVelocity
- Unity XR CommonUsages: https://docs.unity3d.com/ScriptReference/XR.CommonUsages.html
- Unity XRNodeState: https://docs.unity3d.com/ScriptReference/XR.XRNodeState.html
- Quest Passthrough Camera API samples: https://github.com/oculus-samples/Unity-PassthroughCameraApiSamples
- ROS2 rosbag2: https://github.com/ros2/rosbag2
- MCAP ROS2 guide: https://mcap.dev/guides/getting-started/ros-2
