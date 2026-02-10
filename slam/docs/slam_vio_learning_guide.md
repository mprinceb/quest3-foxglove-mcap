# SLAM/VIO Learning Guide for This Repository

## Core Concepts
## SLAM
Simultaneous Localization and Mapping estimates camera/device trajectory while building a map of the environment.

## VIO
Visual-Inertial Odometry combines camera + IMU. It usually improves robustness during fast motion, blur, and low texture.

## EuRoC
EuRoC is a dataset format convention used by many SLAM/VIO tools. In practice it means a standardized directory layout for camera/IMU streams and calibration files.

## Basalt / OpenVINS / VINS-Fusion
- Basalt: modern visual-inertial stack with calibration/mapping tooling.
- OpenVINS: EKF/MSCKF-style VIO framework with strong research focus.
- VINS-Fusion: optimization-based visual-inertial and multi-sensor framework.

They are full external SLAM/VIO systems. This repo prepares data and evaluation so they can be tested consistently.

## What We Implemented Here
Current implemented path:
1. `check-capture`: data quality checks (timestamps, schema, rates)
2. `export-euroc`: Quest data -> EuRoC structure
3. optional estimator:
   - `run-depth-pnp` (in-repo baseline VO)
   - `run-open3d-rgbd` (Open3D RGB-D odometry baseline)
4. `euroc-to-mcap`: EuRoC outputs -> ROS2-profile MCAP for Foxglove/ROS2 playback
5. `pipeline`: single command orchestrating the above

So our default pipeline is:
`data -> EuRoC -> (estimation) -> MCAP`

## Why This Design
- Keeps dataset conversion deterministic and algorithm-agnostic.
- Lets us compare estimators on the same prepared inputs.
- Makes outputs visualization-ready in Foxglove.
- Keeps a migration path to external stacks (ORB-SLAM3, OpenVINS, VINS-Fusion, Basalt).

## Algorithms In This Repo vs External
In-repo:
- Depth-PnP VO: fast sanity baseline, not full loop-closing SLAM.
- Open3D RGB-D odometry: dense-depth odometry baseline, useful for geometry-driven experiments.

External (via submodules/tools):
- ORB-SLAM3/OpenVINS/VINS-Fusion/Basalt for production-grade SLAM/VIO comparisons.

## How to Choose
- Quick pipeline sanity: `depth_pnp`
- Depth-centric experimentation: `open3d`
- Benchmarking serious VIO/SLAM: ORB-SLAM3/OpenVINS/Basalt/VINS-Fusion on EuRoC export
