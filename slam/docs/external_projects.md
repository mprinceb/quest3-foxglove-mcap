# External SLAM/VIO Projects (Git Submodules)

The repository includes these submodules under `third_party/`:

- `third_party/ORB_SLAM3`
  - Purpose: stereo / stereo-inertial SLAM baseline.
- `third_party/open_vins`
  - Purpose: visual-inertial estimator benchmark.
- `third_party/VINS-Fusion`
  - Purpose: optimization-based visual-inertial benchmark.
- `third_party/basalt`
  - Purpose: visual-inertial pipeline and calibration workflows.
- `third_party/rtabmap_ros`
  - Purpose: ROS2 mapping/localization stack integration.

## Clone/Update Notes
After cloning this repo:
```bash
git submodule update --init --recursive
```

To pull latest submodule heads (when desired):
```bash
git submodule update --remote --recursive
```

## Why Submodules Here
They give stable commit-pinned dependencies so comparisons are reproducible across runs.
