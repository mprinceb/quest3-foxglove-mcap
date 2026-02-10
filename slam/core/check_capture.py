"""Validate Quest 3 capture sessions for SLAM/VIO ingestion."""

from __future__ import annotations

import argparse
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from slam.core.quest_dataset import load_csv_rows


@dataclass
class TimeStats:
    count: int
    regressions: int
    duplicates: int
    min_dt: float | None
    max_dt: float | None
    median_dt: float | None
    rate_hz: float | None


@dataclass
class CaptureCheckResult:
    session: Path
    warnings: list[str]
    failures: list[str]
    stats: dict[str, TimeStats]
    counts: dict[str, int]


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


def run_capture_checks(session_dir: Path) -> CaptureCheckResult:
    session = session_dir.expanduser().resolve()
    if not session.exists():
        raise FileNotFoundError(f"Session directory not found: {session}")

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
        if not (session / rel).exists():
            failures.append(f"Missing required path: {rel}")

    if failures:
        return CaptureCheckResult(session=session, warnings=warnings, failures=failures, stats={}, counts={})

    imu_rows = load_csv_rows(session / "imu.csv")
    hmd_rows = load_csv_rows(session / "hmd_poses.csv")
    lctrl_rows = load_csv_rows(session / "left_controller_poses.csv")
    rctrl_rows = load_csv_rows(session / "right_controller_poses.csv")
    ldesc_rows = load_csv_rows(session / "left_depth_descriptors.csv")
    rdesc_rows = load_csv_rows(session / "right_depth_descriptors.csv")

    imu_cols = list(imu_rows[0].keys()) if imu_rows else []
    hmd_cols = list(hmd_rows[0].keys()) if hmd_rows else []
    lctrl_cols = list(lctrl_rows[0].keys()) if lctrl_rows else []
    rctrl_cols = list(rctrl_rows[0].keys()) if rctrl_rows else []
    ldesc_cols = list(ldesc_rows[0].keys()) if ldesc_rows else []
    rdesc_cols = list(rdesc_rows[0].keys()) if rdesc_rows else []

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

    left_yuv = folder_ms_stems(session / "left_camera_raw", "yuv")
    right_yuv = folder_ms_stems(session / "right_camera_raw", "yuv")
    left_depth = folder_ms_stems(session / "left_depth", "raw")
    right_depth = folder_ms_stems(session / "right_depth", "raw")

    stats = {
        "imu": analyze_times(parse_float(r["unix_time"]) for r in imu_rows),
        "hmd": analyze_times(parse_float(r["unix_time"]) for r in hmd_rows),
        "left_controller": analyze_times(parse_float(r["unix_time"]) for r in lctrl_rows),
        "right_controller": analyze_times(parse_float(r["unix_time"]) for r in rctrl_rows),
        "left_cam": analyze_times(float(v) for v in left_yuv),
        "right_cam": analyze_times(float(v) for v in right_yuv),
        "left_depth": analyze_times(float(v) for v in left_depth),
        "right_depth": analyze_times(float(v) for v in right_depth),
    }

    ldesc_ts = sorted(int(float(r["timestamp_ms"])) for r in ldesc_rows)
    rdesc_ts = sorted(int(float(r["timestamp_ms"])) for r in rdesc_rows)

    if set(ldesc_ts) != set(left_depth):
        warnings.append(
            "left depth descriptor mismatch: "
            f"missing_in_desc={len(set(left_depth) - set(ldesc_ts))} "
            f"missing_in_depth={len(set(ldesc_ts) - set(left_depth))}"
        )

    if set(rdesc_ts) != set(right_depth):
        warnings.append(
            "right depth descriptor mismatch: "
            f"missing_in_desc={len(set(right_depth) - set(rdesc_ts))} "
            f"missing_in_depth={len(set(rdesc_ts) - set(right_depth))}"
        )

    if stats["imu"].rate_hz is not None and stats["imu"].rate_hz < 80.0:
        warnings.append(f"IMU median rate is low for VIO: {stats['imu'].rate_hz:.2f} Hz")

    for name in ["imu", "hmd", "left_controller", "right_controller"]:
        s = stats[name]
        if s.regressions > 0:
            failures.append(f"{name} timestamp regressions: {s.regressions}")
        if s.duplicates > 0:
            warnings.append(f"{name} duplicate timestamps: {s.duplicates}")

    counts = {
        "left_camera_raw": len(left_yuv),
        "right_camera_raw": len(right_yuv),
        "left_depth": len(left_depth),
        "right_depth": len(right_depth),
        "imu_rows": len(imu_rows),
        "hmd_rows": len(hmd_rows),
        "stereo_matched": len(set(left_yuv) & set(right_yuv)),
        "stereo_left_only": len(set(left_yuv) - set(right_yuv)),
        "stereo_right_only": len(set(right_yuv) - set(left_yuv)),
    }

    return CaptureCheckResult(session=session, warnings=warnings, failures=failures, stats=stats, counts=counts)


def print_capture_check_result(result: CaptureCheckResult) -> None:
    print(f"Session: {result.session}")
    if result.failures and not result.stats:
        print("\nFailures:")
        for f in result.failures:
            print(f"- {f}")
        return

    print("\nCounts:")
    print(f"- left_camera_raw: {result.counts['left_camera_raw']}")
    print(f"- right_camera_raw: {result.counts['right_camera_raw']}")
    print(f"- left_depth: {result.counts['left_depth']}")
    print(f"- right_depth: {result.counts['right_depth']}")
    print(f"- imu.csv rows: {result.counts['imu_rows']}")
    print(f"- hmd_poses.csv rows: {result.counts['hmd_rows']}")
    print(
        f"- stereo matched timestamps: {result.counts['stereo_matched']} "
        f"(left-only={result.counts['stereo_left_only']}, right-only={result.counts['stereo_right_only']})"
    )

    print("\nTiming:")
    for label in [
        "imu",
        "hmd",
        "left_controller",
        "right_controller",
        "left_cam",
        "right_cam",
        "left_depth",
        "right_depth",
    ]:
        print_time_report(label, result.stats[label])

    if result.warnings:
        print("\nWarnings:")
        for w in result.warnings:
            print(f"- {w}")

    if result.failures:
        print("\nFailures:")
        for f in result.failures:
            print(f"- {f}")


def add_parser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser = subparsers.add_parser("check-capture", help="Validate Quest capture session health")
    parser.add_argument("--session-dir", required=True, help="Path like data/20260209_173413")
    parser.add_argument("--strict", action="store_true", help="Return non-zero if warnings are present")
    return parser


def run_cli(args: argparse.Namespace) -> int:
    result = run_capture_checks(Path(args.session_dir))
    print_capture_check_result(result)
    if args.strict and (result.warnings or result.failures):
        return 2
    return 1 if result.failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Quest 3 capture consistency for SLAM/VIO")
    parser.add_argument("--session-dir", required=True, help="Path like data/20260209_173413")
    parser.add_argument("--strict", action="store_true", help="Return non-zero for warnings/failures")
    args = parser.parse_args()
    return run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
