# Open3D vs EuRoC in This Project

## Not Direct Substitutes
- **EuRoC**: data format/contract for camera+IMU datasets.
- **Open3D**: algorithm/library toolkit for geometry, odometry, registration, reconstruction.

So the right comparison is:
- EuRoC as the intermediate dataset interface
- Open3D as one estimator path consuming EuRoC exports

## Why Keep EuRoC
1. Interoperability with ORB-SLAM3, OpenVINS, VINS-Fusion, Basalt workflows.
2. Standardized camera/IMU directory contracts.
3. Easy benchmarking and reproducibility across estimators.

## Why Add Open3D
1. Fast prototyping for RGB-D odometry and reconstruction.
2. Useful for depth-centric experiments without ROS dependencies.
3. Good debugging visibility for geometry and frame-to-frame behavior.

## Tradeoff Summary
- **EuRoC path strengths**: benchmarkability, tool compatibility, cleaner estimator swaps.
- **Open3D strengths**: practical RGB-D experimentation and dense 3D processing.
- **Limitation**: Open3D RGB-D odometry is not a full visual-inertial SLAM substitute by itself.

## Recommended Architecture (Implemented)
`data -> EuRoC export -> estimator (Depth-PnP/Open3D/External SLAM) -> MCAP`

This keeps one stable ingestion layer while allowing multiple algorithms.
