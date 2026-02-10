#!/usr/bin/env python3
"""Validate Quest 3 capture sessions for SLAM/VIO ingestion.

Checks:
- Required files/folders exist
- CSV schema contains expected columns
- Timestamp monotonicity and duplicate counts
- File stem timestamps align with descriptor CSVs
- Sensor rates and overlap diagnostics
"""

from __future__ import annotations

import argparse
import csv
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass
class TimeStats:
    count: int
    regressions: int
    duplicates: int
    min_dt: float | None
    max_dt: float | None
    median_dt: float | None
    rate_hz: float | None


def read_csv_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        return list(reader.fieldnames or []), rows


def parse_float(value: str) -> float:
    if value in ("", "None", "null"):
        return 0.0
    return float(value)


def analyze_times(values: Iterable[float]) -> TimeStats:
    vals = list(values)
    if len(vals) <= 1:
        return TimeStats(len(vals), 0, 0, None, None, None, None)

    regressions = 0
    duplicates = 0
    deltas = []
    for prev, curr in zip(vals, vals[1:]):
        dt = curr - prev
        if dt < 0:
            regressions += 1
        elif dt == 0:
            duplicates += 1
        else:
            deltas.append(dt)

    if not deltas:
        return TimeStats(len(vals), regressions, duplicates, None, None, None, None)

    min_dt = min(deltas)
    max_dt = max(deltas)
    med_dt = statistics.median(deltas)
    rate_hz = 1000.0 / med_dt if med_dt > 0 else None
    return TimeStats(len(vals), regressions, duplicates, min_dt, max_dt, med_dt, rate_hz)


def folder_ms_stems(folder: Path, suffix: str) -> list[int]:
    return sorted(int(p.stem) for p in folder.glob(f"*.{suffix}"))


def require_columns(name: str, columns: list[str], required: list[str], failures: list[str]) -> None:
    missing = [c for c in required if c not in columns]
    if missing:
        failures.append(f"{name}: missing required columns: {missing}")


def print_time_report(label: str, stats: TimeStats) -> None:
    print(f"[{label}] count={stats.count} regressions={stats.regressions} duplicates={stats.duplicates}")
    if stats.median_dt is not None:
        print(
            f"  dt_ms(min/med/max)=({stats.min_dt:.3f}/{stats.median_dt:.3f}/{stats.max_dt:.3f})"
            f" rate_hz~{stats.rate_hz:.2f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Quest 3 capture consistency for SLAM/VIO")
    parser.add_argument("--session-dir", required=True, help="Path like data/20260209_173413")
    parser.add_argument("--strict", action="store_true", help="Return non-zero for any warning/failure")
    args = parser.parse_args()

    session = Path(args.session_dir).expanduser().resolve()
    if not session.exists():
        raise FileNotFoundError(f"Session directory not found: {session}")

    print(f"Session: {session}")

    failures: list[str] = []
    warnings: list[str] = []

    required_paths = [
        "left_camera_raw",
        "right_camera_raw",
        "left_depth",
        "right_depth",
        "imu.csv",
        "hmd_poses.csv",
        "left_controller_poses.csv",
        "right_controller_poses.csv",
        "left_depth_descriptors.csv",
        "right_depth_descriptors.csv",
        "left_camera_characteristics.json",
        "right_camera_characteristics.json",
        "left_camera_image_format.json",
        "right_camera_image_format.json",
    ]

    for rel in required_paths:
        p = session / rel
        if not p.exists():
            failures.append(f"Missing required path: {rel}")

    if failures:
        print("\nFailures:")
        for f in failures:
            print(f"- {f}")
        return 1

    imu_cols, imu_rows = read_csv_rows(session / "imu.csv")
    hmd_cols, hmd_rows = read_csv_rows(session / "hmd_poses.csv")
    lctrl_cols, lctrl_rows = read_csv_rows(session / "left_controller_poses.csv")
    rctrl_cols, rctrl_rows = read_csv_rows(session / "right_controller_poses.csv")
    ldesc_cols, ldesc_rows = read_csv_rows(session / "left_depth_descriptors.csv")
    rdesc_cols, rdesc_rows = read_csv_rows(session / "right_depth_descriptors.csv")

    require_columns(
        "imu.csv",
        imu_cols,
        ["unix_time", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"],
        failures,
    )
    require_columns(
        "hmd_poses.csv",
        hmd_cols,
        ["unix_time", "pos_x", "pos_y", "pos_z", "rot_x", "rot_y", "rot_z", "rot_w"],
        failures,
    )
    require_columns(
        "left_controller_poses.csv",
        lctrl_cols,
        ["unix_time", "pos_x", "pos_y", "pos_z", "rot_x", "rot_y", "rot_z", "rot_w"],
        failures,
    )
    require_columns(
        "right_controller_poses.csv",
        rctrl_cols,
        ["unix_time", "pos_x", "pos_y", "pos_z", "rot_x", "rot_y", "rot_z", "rot_w"],
        failures,
    )
    require_columns("left_depth_descriptors.csv", ldesc_cols, ["timestamp_ms", "width", "height"], failures)
    require_columns("right_depth_descriptors.csv", rdesc_cols, ["timestamp_ms", "width", "height"], failures)

    if failures:
        print("\nFailures:")
        for f in failures:
            print(f"- {f}")
        return 1

    left_yuv = folder_ms_stems(session / "left_camera_raw", "yuv")
    right_yuv = folder_ms_stems(session / "right_camera_raw", "yuv")
    left_depth = folder_ms_stems(session / "left_depth", "raw")
    right_depth = folder_ms_stems(session / "right_depth", "raw")

    print("\nCounts:")
    print(f"- left_camera_raw: {len(left_yuv)}")
    print(f"- right_camera_raw: {len(right_yuv)}")
    print(f"- left_depth: {len(left_depth)}")
    print(f"- right_depth: {len(right_depth)}")
    print(f"- imu.csv rows: {len(imu_rows)}")
    print(f"- hmd_poses.csv rows: {len(hmd_rows)}")

    left_only = len(set(left_yuv) - set(right_yuv))
    right_only = len(set(right_yuv) - set(left_yuv))
    common_stereo = len(set(left_yuv) & set(right_yuv))
    print(f"- stereo matched timestamps: {common_stereo} (left-only={left_only}, right-only={right_only})")

    imu_stats = analyze_times(parse_float(r["unix_time"]) for r in imu_rows)
    hmd_stats = analyze_times(parse_float(r["unix_time"]) for r in hmd_rows)
    lctrl_stats = analyze_times(parse_float(r["unix_time"]) for r in lctrl_rows)
    rctrl_stats = analyze_times(parse_float(r["unix_time"]) for r in rctrl_rows)
    lcam_stats = analyze_times(float(v) for v in left_yuv)
    rcam_stats = analyze_times(float(v) for v in right_yuv)
    ldepth_stats = analyze_times(float(v) for v in left_depth)
    rdepth_stats = analyze_times(float(v) for v in right_depth)

    print("\nTiming:")
    print_time_report("imu", imu_stats)
    print_time_report("hmd", hmd_stats)
    print_time_report("left_controller", lctrl_stats)
    print_time_report("right_controller", rctrl_stats)
    print_time_report("left_cam", lcam_stats)
    print_time_report("right_cam", rcam_stats)
    print_time_report("left_depth", ldepth_stats)
    print_time_report("right_depth", rdepth_stats)

    ldesc_ts = sorted(int(float(r["timestamp_ms"])) for r in ldesc_rows)
    rdesc_ts = sorted(int(float(r["timestamp_ms"])) for r in rdesc_rows)

    if set(ldesc_ts) != set(left_depth):
        missing_in_desc = sorted(set(left_depth) - set(ldesc_ts))
        missing_in_depth = sorted(set(ldesc_ts) - set(left_depth))
        warnings.append(
            "left depth descriptor mismatch: "
            f"missing_in_desc={len(missing_in_desc)} missing_in_depth={len(missing_in_depth)}"
        )

    if set(rdesc_ts) != set(right_depth):
        missing_in_desc = sorted(set(right_depth) - set(rdesc_ts))
        missing_in_depth = sorted(set(rdesc_ts) - set(right_depth))
        warnings.append(
            "right depth descriptor mismatch: "
            f"missing_in_desc={len(missing_in_desc)} missing_in_depth={len(missing_in_depth)}"
        )

    if imu_stats.rate_hz is not None and imu_stats.rate_hz < 80.0:
        warnings.append(f"IMU median rate is low for VIO: {imu_stats.rate_hz:.2f} Hz")

    for name, stats in [
        ("imu", imu_stats),
        ("hmd", hmd_stats),
        ("left_controller", lctrl_stats),
        ("right_controller", rctrl_stats),
    ]:
        if stats.regressions > 0:
            failures.append(f"{name} timestamp regressions: {stats.regressions}")
        if stats.duplicates > 0:
            warnings.append(f"{name} duplicate timestamps: {stats.duplicates}")

    if warnings:
        print("\nWarnings:")
        for w in warnings:
            print(f"- {w}")

    if failures:
        print("\nFailures:")
        for f in failures:
            print(f"- {f}")

    if args.strict and (warnings or failures):
        return 2
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
