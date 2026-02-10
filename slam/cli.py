"""Unified CLI for Quest3 SLAM/VIO pipelines and tools."""

from __future__ import annotations

import argparse

from slam.adapters import euroc_to_mcap, export_euroc, export_tum
from slam.algorithms import depth_pnp_vo, open3d_rgbd_odometry, orbslam3, rtabmap
from slam.core import check_capture, pipeline
from slam.eval import evaluate_evo


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="quest-slam",
        description="Quest3 SLAM/VIO toolbox: QA, data conversion, estimators, and visualization helpers.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    check_capture.add_parser(subparsers)
    export_euroc.add_parser(subparsers)
    export_tum.add_parser(subparsers)
    euroc_to_mcap.add_parser(subparsers)
    depth_pnp_vo.add_parser(subparsers)
    open3d_rgbd_odometry.add_parser(subparsers)
    pipeline.add_parser(subparsers)
    evaluate_evo.add_parser(subparsers)
    orbslam3.add_parser(subparsers)
    rtabmap.add_parser(subparsers)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        if args.command == "check-capture":
            return check_capture.run_cli(args)
        if args.command == "export-euroc":
            return export_euroc.run_cli(args)
        if args.command == "export-tum":
            return export_tum.run_cli(args)
        if args.command == "euroc-to-mcap":
            return euroc_to_mcap.run_cli(args)
        if args.command == "run-depth-pnp":
            return depth_pnp_vo.run_cli(args)
        if args.command == "run-open3d-rgbd":
            return open3d_rgbd_odometry.run_cli(args)
        if args.command == "pipeline":
            return pipeline.run_cli(args)
        if args.command == "evaluate-evo":
            return evaluate_evo.run_cli(args)
        if args.command in {"orbslam3-config", "run-orbslam3"}:
            return orbslam3.run_cli(args)
        if args.command == "run-rtabmap":
            return rtabmap.run_cli(args)
        parser.error(f"Unknown command: {args.command}")
        return 2
    except Exception as exc:
        parser.exit(1, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
