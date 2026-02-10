#!/usr/bin/env python3
"""Export Quest 3 pose CSVs to TUM trajectory format."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def write_tum(input_csv: Path, output_txt: Path) -> int:
    count = 0
    with input_csv.open(newline="") as f_in, output_txt.open("w") as f_out:
        r = csv.DictReader(f_in)
        for row in r:
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Export hmd/controller pose CSV to TUM format")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    session = Path(args.session_dir).expanduser().resolve()
    out = Path(args.out_dir).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    pairs = [
        (session / "hmd_poses.csv", out / "hmd.tum.txt"),
        (session / "left_controller_poses.csv", out / "left_controller.tum.txt"),
        (session / "right_controller_poses.csv", out / "right_controller.tum.txt"),
    ]

    for src, dst in pairs:
        if not src.exists():
            print(f"skip missing: {src}")
            continue
        count = write_tum(src, dst)
        print(f"wrote {dst} ({count} rows)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
