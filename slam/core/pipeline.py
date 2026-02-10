"""High-level Quest session SLAM pipeline orchestration."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from slam.adapters.euroc_to_mcap import convert_euroc_to_mcap
from slam.adapters.export_euroc import export_session_to_euroc
from slam.adapters.export_tum import export_session_to_tum
from slam.algorithms.depth_pnp_vo import run_depth_pnp_vo
from slam.algorithms.open3d_rgbd_odometry import run_open3d_rgbd_odometry
from slam.core.check_capture import print_capture_check_result, run_capture_checks


@dataclass
class PipelineSummary:
    session: str
    euroc_dir: str
    mcap: str
    estimated_tum: str | None
    estimator: str


def run_pipeline(
    session_dir: Path,
    *,
    work_dir: Path,
    mcap_out: Path | None = None,
    max_frames: int = 0,
    include_depth: bool = False,
    image_mode: str = "mono",
    skip_qa: bool = False,
    estimator: str = "depth_pnp",
) -> PipelineSummary:
    session_dir = session_dir.expanduser().resolve()
    if not session_dir.exists():
        raise FileNotFoundError(f"Session directory not found: {session_dir}")

    session_id = session_dir.name
    work_dir = work_dir.expanduser().resolve()
    work_dir.mkdir(parents=True, exist_ok=True)

    euroc_dir = work_dir / "euroc"
    ref_dir = work_dir / "reference"
    ref_dir.mkdir(parents=True, exist_ok=True)

    final_mcap = mcap_out.expanduser().resolve() if mcap_out else (work_dir / f"{session_id}_slam.mcap")

    if not skip_qa:
        check = run_capture_checks(session_dir)
        print_capture_check_result(check)
        if check.failures:
            raise RuntimeError("Capture QA reported failures")

    export_session_to_euroc(
        session_dir,
        euroc_dir,
        max_frames=max_frames,
        include_depth=include_depth,
        image_mode=image_mode,
    )

    export_session_to_tum(session_dir, ref_dir)

    estimated_tum: Path | None = None
    if estimator != "none":
        if estimator == "depth_pnp":
            estimated_tum = work_dir / "estimated_depth_pnp.tum.txt"
            run_depth_pnp_vo(
                session_dir,
                estimated_tum,
                max_frames=max_frames,
                image_mode=image_mode,
            )
        elif estimator == "open3d":
            estimated_tum = work_dir / "estimated_open3d_rgbd.tum.txt"
            run_open3d_rgbd_odometry(
                euroc_dir,
                estimated_tum,
                max_frames=max_frames,
            )
        else:
            raise ValueError(f"Unknown estimator: {estimator}")

    convert_euroc_to_mcap(
        euroc_dir,
        final_mcap,
        estimated_tum=estimated_tum,
        reference_tum=euroc_dir / "mav0" / "state_groundtruth_estimate0" / "tum.txt",
        include_depth=include_depth,
        max_frames=max_frames,
        image_mode="mono" if image_mode == "mono" else "rgb",
    )

    return PipelineSummary(
        session=str(session_dir),
        euroc_dir=str(euroc_dir),
        mcap=str(final_mcap),
        estimated_tum=str(estimated_tum) if estimated_tum and estimated_tum.exists() else None,
        estimator=estimator,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Quest3 data -> EuRoC -> MCAP pipeline")
    parser.add_argument("--session-dir", required=True, help="Input session dir (data/<session_id>)")
    parser.add_argument(
        "--work-dir",
        default="",
        help="Pipeline work/output directory (default: slam/results/pipeline_<session_id>)",
    )
    parser.add_argument("--mcap-out", default="", help="Final MCAP output path")
    parser.add_argument("--max-frames", type=int, default=0, help="Limit stereo frames for quick runs")
    parser.add_argument("--include-depth", action="store_true", help="Include depth topics in EuRoC + MCAP")
    parser.add_argument("--skip-qa", action="store_true", help="Skip capture QA checks")
    parser.add_argument("--image-mode", choices=["mono", "rgb"], default="mono")
    parser.add_argument(
        "--estimator",
        choices=["none", "depth_pnp", "open3d"],
        default="depth_pnp",
        help="Trajectory estimator to run before MCAP export.",
    )
    return parser.parse_args()


def add_parser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser = subparsers.add_parser("pipeline", help="Run full data->EuRoC->MCAP pipeline")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--work-dir", default="")
    parser.add_argument("--mcap-out", default="")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--include-depth", action="store_true")
    parser.add_argument("--skip-qa", action="store_true")
    parser.add_argument("--image-mode", choices=["mono", "rgb"], default="mono")
    parser.add_argument("--estimator", choices=["none", "depth_pnp", "open3d"], default="depth_pnp")
    return parser


def run_cli(args: argparse.Namespace) -> int:
    session_dir = Path(args.session_dir).expanduser().resolve()
    session_id = session_dir.name
    repo_root = Path(__file__).resolve().parents[2]

    work_dir = (
        Path(args.work_dir).expanduser().resolve()
        if args.work_dir
        else (repo_root / "slam" / "results" / f"pipeline_{session_id}")
    )

    summary = run_pipeline(
        session_dir,
        work_dir=work_dir,
        mcap_out=Path(args.mcap_out).expanduser().resolve() if args.mcap_out else None,
        max_frames=args.max_frames,
        include_depth=args.include_depth,
        image_mode=args.image_mode,
        skip_qa=args.skip_qa,
        estimator=args.estimator,
    )

    print("\nPipeline complete")
    print(f"session: {summary.session}")
    print(f"euroc:   {summary.euroc_dir}")
    print(f"mcap:    {summary.mcap}")
    if summary.estimated_tum:
        print(f"est:     {summary.estimated_tum} ({summary.estimator})")
    print("Open the MCAP in Foxglove for visualization.")
    return 0


def main() -> int:
    args = parse_args()
    return run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
