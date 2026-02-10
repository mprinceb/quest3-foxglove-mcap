"""Export Quest 3 pose CSVs to TUM trajectory format."""

from __future__ import annotations

import argparse
from pathlib import Path

from slam.core.quest_dataset import load_csv_rows


def write_tum(input_csv: Path, output_txt: Path) -> int:
    rows = load_csv_rows(input_csv)
    count = 0
    with output_txt.open("w") as f_out:
        for row in rows:
            try:
                t_s = float(row["unix_time"]) / 1000.0
                px = float(row["pos_x"])
                py = float(row["pos_y"])
                pz = float(row["pos_z"])
                qx = float(row["rot_x"])
                qy = float(row["rot_y"])
                qz = float(row["rot_z"])
                qw = float(row["rot_w"])
            except (KeyError, ValueError):
                continue
            f_out.write(f"{t_s:.9f} {px:.9f} {py:.9f} {pz:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
            count += 1
    return count


def export_session_to_tum(session_dir: Path, out_dir: Path) -> dict[str, int]:
    session = session_dir.expanduser().resolve()
    out = out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    pairs = [
        (session / "hmd_poses.csv", out / "hmd.tum.txt"),
        (session / "left_controller_poses.csv", out / "left_controller.tum.txt"),
        (session / "right_controller_poses.csv", out / "right_controller.tum.txt"),
    ]

    summary: dict[str, int] = {}
    for src, dst in pairs:
        if not src.exists():
            print(f"skip missing: {src}")
            continue
        count = write_tum(src, dst)
        summary[dst.name] = count
        print(f"wrote {dst} ({count} rows)")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export hmd/controller pose CSV to TUM format")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    return parser.parse_args()


def add_parser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser = subparsers.add_parser("export-tum", help="Export HMD/controller poses to TUM text files")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    return parser


def run_cli(args: argparse.Namespace) -> int:
    export_session_to_tum(Path(args.session_dir), Path(args.out_dir))
    return 0


def main() -> int:
    args = parse_args()
    return run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
