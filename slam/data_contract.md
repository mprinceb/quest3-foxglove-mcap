# SLAM Data Contract (Quest 3 Capture)

## Scope
This contract defines the canonical session schema consumed by `slam/` tools.

## Session Directory Layout
Required paths under `data/<session_id>/`:
- `left_camera_raw/*.yuv`
- `right_camera_raw/*.yuv`
- `left_depth/*.raw`
- `right_depth/*.raw`
- `imu.csv`
- `hmd_poses.csv`
- `left_controller_poses.csv`
- `right_controller_poses.csv`
- `left_depth_descriptors.csv`
- `right_depth_descriptors.csv`
- `left_camera_image_format.json`
- `right_camera_image_format.json`
- `left_camera_characteristics.json`
- `right_camera_characteristics.json`

## Time and Units
- `unix_time` / `timestamp_ms`: integer milliseconds since Unix epoch.
- Internal SLAM tooling converts all timestamps to nanoseconds (`ns`).
- IMU gyro units: `rad/s` (`gyro_x`, `gyro_y`, `gyro_z`).
- IMU accel units: `m/s^2` (`acc_x`, `acc_y`, `acc_z`).
- Pose positions: meters.
- Pose rotations: quaternion `xyzw`.

## CSV Field Expectations
### `imu.csv`
Required: `unix_time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z`
Optional diagnostic fields currently used when present: `vel_*`, `ang_acc_*`, `ovr_timestamp`.

### `hmd_poses.csv`, `left_controller_poses.csv`, `right_controller_poses.csv`
Required: `unix_time, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w`

### `left_depth_descriptors.csv`, `right_depth_descriptors.csv`
Required: `timestamp_ms, width, height`
Expected: `create_pose_*`, FOV tangents, near/far clipping.

## Consistency Rules
- Timestamps must be monotonic non-decreasing per stream.
- Stereo SLAM export uses exact timestamp intersection of left/right YUV frame names.
- Depth `.raw` stem timestamps should match corresponding descriptor rows.
- Missing samples are allowed; regressions in time are treated as failures.
