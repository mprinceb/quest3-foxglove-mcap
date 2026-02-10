"""ORB-SLAM3 helpers (config generation + run wrappers)."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import numpy as np

from slam.adapters.euroc_to_mcap import parse_sensor_yaml


def generate_orbslam3_config(euroc_dir: Path, out_yaml: Path, *, image_mode: str = "mono") -> Path:
    euroc_dir = euroc_dir.expanduser().resolve()
    mav0 = euroc_dir / "mav0" if (euroc_dir / "mav0").exists() else euroc_dir

    cam0 = parse_sensor_yaml(mav0 / "cam0" / "sensor.yaml")
    cam1 = parse_sensor_yaml(mav0 / "cam1" / "sensor.yaml")

    baseline_m = float(np.linalg.norm(np.array(cam1.t_bs, dtype=float)))
    bf = cam0.fx * baseline_m

    # derive approximate fps from orbslam times if present
    fps = 30.0
    times_file = mav0 / "meta" / "orbslam_times.txt"
    if times_file.exists():
        vals = []
        for line in times_file.read_text().splitlines():
            try:
                vals.append(float(line.strip()))
            except ValueError:
                continue
        if len(vals) > 1:
            dts = [b - a for a, b in zip(vals, vals[1:]) if b > a]
            if dts:
                fps = 1.0 / float(np.median(np.array(dts)))

    rgb_flag = 1 if image_mode == "rgb" else 0

    yaml = f"""%YAML:1.0

File.version: "1.0"
Camera.type: "PinHole"
Camera.fx: {cam0.fx:.9f}
Camera.fy: {cam0.fy:.9f}
Camera.cx: {cam0.cx:.9f}
Camera.cy: {cam0.cy:.9f}
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.k3: 0.0
Camera.width: {cam0.width}
Camera.height: {cam0.height}
Camera.fps: {fps:.6f}
Camera.bf: {bf:.9f}
Camera.RGB: {rgb_flag}
ThDepth: 40.0
DepthMapFactor: 1.0
ORBextractor.nFeatures: 1800
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
"""

    out_yaml = out_yaml.expanduser().resolve()
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    out_yaml.write_text(yaml)
    return out_yaml


def run_orbslam3(
    euroc_dir: Path,
    *,
    mode: str,
    config_yaml: Path,
    results_dir: Path,
    orb_slam3_root: str | None,
    orb_vocabulary: str | None,
    orb_binary: str | None,
) -> None:
    euroc_dir = euroc_dir.expanduser().resolve()
    mav0 = euroc_dir / "mav0" if (euroc_dir / "mav0").exists() else euroc_dir
    times_file = mav0 / "meta" / "orbslam_times.txt"

    if not times_file.exists():
        raise RuntimeError(f"Missing times file: {times_file}")

    if mode not in {"stereo", "stereo-inertial"}:
        raise ValueError("mode must be stereo or stereo-inertial")

    if orb_binary:
        binary = Path(orb_binary).expanduser().resolve()
    else:
        if not orb_slam3_root:
            raise RuntimeError("Provide --orb-slam3-root or --orb-binary")
        root = Path(orb_slam3_root).expanduser().resolve()
        rel = "Examples/Stereo/stereo_euroc" if mode == "stereo" else "Examples/Stereo-Inertial/stereo_inertial_euroc"
        binary = root / rel

    if orb_vocabulary:
        vocab = Path(orb_vocabulary).expanduser().resolve()
    else:
        if not orb_slam3_root:
            raise RuntimeError("Provide --orb-slam3-root or --orb-vocabulary")
        vocab = Path(orb_slam3_root).expanduser().resolve() / "Vocabulary" / "ORBvoc.txt"

    config_yaml = config_yaml.expanduser().resolve()
    results_dir = results_dir.expanduser().resolve()
    results_dir.mkdir(parents=True, exist_ok=True)

    for p in [binary, vocab, config_yaml, times_file]:
        if not p.exists():
            raise RuntimeError(f"Missing required path: {p}")

    cmd = [str(binary), str(vocab), str(config_yaml), str(mav0), str(times_file)]
    with (results_dir / "run.log").open("w") as f:
        subprocess.run(cmd, check=True, cwd=str(results_dir), stdout=f, stderr=subprocess.STDOUT)

    for fname in ["CameraTrajectory.txt", "KeyFrameTrajectory.txt"]:
        src = results_dir / fname
        if src.exists():
            (results_dir / f"{fname[:-4]}.tum.txt").write_text(src.read_text())


def add_parser(subparsers: argparse._SubParsersAction) -> None:
    p1 = subparsers.add_parser("orbslam3-config", help="Generate ORB-SLAM3 config from EuRoC sensor metadata")
    p1.add_argument("--euroc-dir", required=True)
    p1.add_argument("--out-yaml", required=True)
    p1.add_argument("--image-mode", choices=["mono", "rgb"], default="mono")

    p2 = subparsers.add_parser("run-orbslam3", help="Run ORB-SLAM3 stereo or stereo-inertial")
    p2.add_argument("--euroc-dir", required=True)
    p2.add_argument("--mode", choices=["stereo", "stereo-inertial"], default="stereo")
    p2.add_argument("--config-yaml", required=True)
    p2.add_argument("--results-dir", required=True)
    p2.add_argument("--orb-slam3-root", default="")
    p2.add_argument("--orb-vocabulary", default="")
    p2.add_argument("--orb-binary", default="")


def run_cli(args: argparse.Namespace) -> int:
    if args.command == "orbslam3-config":
        out = generate_orbslam3_config(Path(args.euroc_dir), Path(args.out_yaml), image_mode=args.image_mode)
        print(f"Generated ORB-SLAM3 config: {out}")
        return 0

    if args.command == "run-orbslam3":
        run_orbslam3(
            Path(args.euroc_dir),
            mode=args.mode,
            config_yaml=Path(args.config_yaml),
            results_dir=Path(args.results_dir),
            orb_slam3_root=args.orb_slam3_root or None,
            orb_vocabulary=args.orb_vocabulary or None,
            orb_binary=args.orb_binary or None,
        )
        print(f"ORB-SLAM3 run complete: {Path(args.results_dir).expanduser().resolve()}")
        return 0

    raise ValueError(f"Unsupported command: {args.command}")
