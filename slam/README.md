# SLAM Python Module

`slam/` is now a Python module with a single CLI entrypoint.

## Module Structure
- `slam/cli.py`: unified CLI (`quest-slam`)
- `slam/core/`: capture QA + end-to-end pipeline orchestration
- `slam/adapters/`: `data -> EuRoC`, `EuRoC -> MCAP`, pose exports
- `slam/algorithms/`: in-repo estimators (Depth-PnP, Open3D) + external runner helpers
- `slam/eval/`: trajectory evaluation helpers
- `slam/docs/`: contracts, RTAB-map notes, learning guides

## CLI Entry Point
Primary entrypoint:
```bash
uv run python -m slam.cli --help
```

## Single Command Pipeline (Data -> EuRoC -> MCAP)
```bash
uv run python -m slam.cli pipeline \
  --session-dir data/20260209_173413 \
  --include-depth \
  --image-mode mono \
  --estimator depth_pnp
```

Key options:
- `--image-mode mono|rgb`: choose grayscale or color image export/publishing
- `--estimator none|depth_pnp|open3d`: estimator before MCAP packaging
- `--max-frames N`: smoke-test limit

Output:
- `slam/results/pipeline_<session>/<session>_slam.mcap`

## Common Subcommands
```bash
uv run python -m slam.cli check-capture --session-dir data/20260209_173413
uv run python -m slam.cli export-euroc --session-dir data/20260209_173413 --out-dir slam/results/euroc_run --image-mode rgb
uv run python -m slam.cli euroc-to-mcap --euroc-dir slam/results/euroc_run --out slam/results/euroc_run/run.mcap --image-mode rgb
uv run python -m slam.cli run-depth-pnp --session-dir data/20260209_173413 --out-tum slam/results/depth_pnp.tum.txt
uv run --with open3d python -m slam.cli run-open3d-rgbd --euroc-dir slam/results/euroc_run --out-tum slam/results/open3d.tum.txt
uv run python -m slam.cli evaluate-evo --ref ref.tum.txt --est est.tum.txt --out-dir slam/results/eval
```

## External Algorithms
To integrate ORB-SLAM3:
```bash
uv run python -m slam.cli orbslam3-config --euroc-dir slam/results/euroc_run --out-yaml slam/results/euroc_run/orbslam3.yaml --image-mode mono
uv run python -m slam.cli run-orbslam3 --euroc-dir slam/results/euroc_run --mode stereo --config-yaml slam/results/euroc_run/orbslam3.yaml --results-dir slam/results/orbslam3
```

Pinned external projects are added as git submodules in `third_party/`:
- `ORB_SLAM3`
- `open_vins`
- `VINS-Fusion`
- `basalt`
- `rtabmap_ros`

## Notes
- Camera/image metadata is read from each dataset run (`*_camera_characteristics.json`, `*_camera_image_format.json`).
- Stereo baseline in `sensor.yaml` is metadata-derived and should be replaced by calibrated extrinsics for production.
- Open3D is optional and may require `uv run --with open3d ...` depending on Python/platform wheels.
