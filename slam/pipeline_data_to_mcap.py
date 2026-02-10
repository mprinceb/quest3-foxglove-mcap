#!/usr/bin/env python3
"""Single-command Quest session pipeline:

data session -> EuRoC export -> optional baseline VO -> MCAP (Foxglove/ROS2 profile)
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


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
    parser.add_argument(
        "--skip-baseline-vo",
        action="store_true",
        help="Skip in-repo depth+PnP VO estimation step",
    )
    return parser.parse_args()


def run_cmd(args: list[str]) -> None:
    print("+", " ".join(args), flush=True)
    subprocess.run(args, check=True)


def main() -> int:
    args = parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    session_dir = Path(args.session_dir).expanduser().resolve()
    if not session_dir.exists():
        raise FileNotFoundError(f"Session directory not found: {session_dir}")

    session_id = session_dir.name
    work_dir = (
        Path(args.work_dir).expanduser().resolve()
        if args.work_dir
        else (repo_root / "slam" / "results" / f"pipeline_{session_id}")
    )
    work_dir.mkdir(parents=True, exist_ok=True)

    euroc_dir = work_dir / "euroc"
    ref_dir = work_dir / "reference"
    ref_dir.mkdir(parents=True, exist_ok=True)

    final_mcap = (
        Path(args.mcap_out).expanduser().resolve()
        if args.mcap_out
        else (work_dir / f"{session_id}_slam.mcap")
    )

    py = sys.executable

    if not args.skip_qa:
        run_cmd([py, str(repo_root / "slam" / "qa" / "check_capture.py"), "--session-dir", str(session_dir)])

    export_cmd = [
        py,
        str(repo_root / "slam" / "adapters" / "export_euroc.py"),
        "--session-dir",
        str(session_dir),
        "--out-dir",
        str(euroc_dir),
    ]
    if args.max_frames > 0:
        export_cmd += ["--max-frames", str(args.max_frames)]
    if args.include_depth:
        export_cmd.append("--include-depth")
    run_cmd(export_cmd)

    run_cmd(
        [
            py,
            str(repo_root / "slam" / "adapters" / "export_tum.py"),
            "--session-dir",
            str(session_dir),
            "--out-dir",
            str(ref_dir),
        ]
    )

    est_tum = work_dir / "estimated_depth_pnp.tum.txt"
    if not args.skip_baseline_vo:
        vo_cmd = [
            py,
            str(repo_root / "slam" / "baselines" / "depth_pnp_vo.py"),
            "--session-dir",
            str(session_dir),
            "--out-tum",
            str(est_tum),
        ]
        if args.max_frames > 0:
            vo_cmd += ["--max-frames", str(args.max_frames)]
        run_cmd(vo_cmd)

    euroc_to_mcap_cmd = [
        py,
        str(repo_root / "slam" / "adapters" / "euroc_to_mcap.py"),
        "--euroc-dir",
        str(euroc_dir),
        "--out",
        str(final_mcap),
        "--reference-tum",
        str(euroc_dir / "mav0" / "state_groundtruth_estimate0" / "tum.txt"),
    ]
    if est_tum.exists():
        euroc_to_mcap_cmd += ["--estimated-tum", str(est_tum)]
    if args.max_frames > 0:
        euroc_to_mcap_cmd += ["--max-frames", str(args.max_frames)]
    if args.include_depth:
        euroc_to_mcap_cmd += ["--include-depth"]
    run_cmd(euroc_to_mcap_cmd)

    print("\nPipeline complete")
    print(f"session: {session_dir}")
    print(f"euroc:   {euroc_dir}")
    print(f"mcap:    {final_mcap}")
    if est_tum.exists():
        print(f"est vo:  {est_tum}")
    print("Open the MCAP in Foxglove for visualization.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
