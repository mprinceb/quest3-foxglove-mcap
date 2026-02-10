"""Evaluate trajectories with evo (APE/RPE)."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def run_evo(ref: Path, est: Path, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    for exe in ["evo_ape", "evo_rpe"]:
        if subprocess.run(["which", exe], capture_output=True).returncode != 0:
            raise RuntimeError("evo CLI not found. Install with: pip install evo --upgrade")

    with (out_dir / "ape.txt").open("w") as ape_out:
        subprocess.run(
            ["evo_ape", "tum", str(ref), str(est), "-va", "--plot", "--save_results", str(out_dir / "ape.zip")],
            check=True,
            stdout=ape_out,
        )

    with (out_dir / "rpe.txt").open("w") as rpe_out:
        subprocess.run(
            ["evo_rpe", "tum", str(ref), str(est), "-va", "--plot", "--save_results", str(out_dir / "rpe.zip")],
            check=True,
            stdout=rpe_out,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate trajectories with evo")
    parser.add_argument("--ref", required=True, help="Reference TUM path")
    parser.add_argument("--est", required=True, help="Estimated TUM path")
    parser.add_argument("--out-dir", required=True)
    return parser.parse_args()


def add_parser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser = subparsers.add_parser("evaluate-evo", help="Run evo APE/RPE on TUM trajectories")
    parser.add_argument("--ref", required=True)
    parser.add_argument("--est", required=True)
    parser.add_argument("--out-dir", required=True)
    return parser


def run_cli(args: argparse.Namespace) -> int:
    run_evo(Path(args.ref).expanduser().resolve(), Path(args.est).expanduser().resolve(), Path(args.out_dir).expanduser().resolve())
    print(f"evo evaluation complete: {Path(args.out_dir).expanduser().resolve()}")
    return 0


def main() -> int:
    args = parse_args()
    return run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
