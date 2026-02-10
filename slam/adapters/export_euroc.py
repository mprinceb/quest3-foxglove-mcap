#!/usr/bin/env python3
"""Export Quest 3 capture session to an EuRoC-style dataset layout.

Inputs expected under session dir:
- left_camera_raw/*.yuv
- right_camera_raw/*.yuv
- imu.csv
- hmd_poses.csv
- *_camera_image_format.json
- *_camera_characteristics.json
- optional depth folders and descriptors

Output layout:
<out>/mav0/
  cam0/data/*.png
  cam0/data.csv
  cam0/sensor.yaml
  cam1/data/*.png
  cam1/data.csv
  cam1/sensor.yaml
  imu0/data.csv
  imu0/sensor.yaml
  depth0|depth1/data/*.png (optional)
  depth0|depth1/data.csv (optional)
  state_groundtruth_estimate0/tum.txt
  meta/session_summary.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export Quest 3 session to EuRoC-style dataset")
    parser.add_argument("--session-dir", required=True, help="Input session dir, e.g. data/20260209_173413")
    parser.add_argument("--out-dir", required=True, help="Output dataset directory")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Limit stereo frames for quick tests (0 = all)",
    )
    parser.add_argument(
        "--include-depth",
        action="store_true",
        help="Export left/right depth .raw into 16-bit PNG sequences",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open() as f:
        return json.load(f)


def load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def ms_to_ns(ts_ms: int) -> int:
    return ts_ms * 1_000_000


def read_y_plane(yuv_path: Path, fmt: dict[str, Any]) -> np.ndarray:
    width = int(fmt["width"])
    height = int(fmt["height"])
    y_row_stride = int(fmt["planes"][0]["rowStride"])
    y_size = int(fmt["planes"][0]["bufferSize"])

    data = np.fromfile(yuv_path, np.uint8)
    if data.size < y_size:
        raise ValueError(f"YUV file too small: {yuv_path} ({data.size} < {y_size})")

    y = data[:y_size].reshape((height, y_row_stride))[:, :width]
    return y


def list_ts_files(folder: Path, suffix: str) -> dict[int, Path]:
    out: dict[int, Path] = {}
    for p in folder.glob(f"*.{suffix}"):
        try:
            ts = int(p.stem)
        except ValueError:
            continue
        out[ts] = p
    return out


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def write_cam_csv(path: Path, rows: list[tuple[int, str]]) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["#timestamp [ns]", "filename"])
        for ts_ns, name in rows:
            w.writerow([ts_ns, name])


def write_orbslam_times(path: Path, ts_ns_values: list[int]) -> None:
    if not ts_ns_values:
        path.write_text("")
        return
    t0 = ts_ns_values[0]
    with path.open("w") as f:
        for ts in ts_ns_values:
            f.write(f"{(ts - t0) / 1e9:.9f}\n")


def write_imu_csv(path: Path, imu_rows: list[dict[str, str]]) -> tuple[int, float]:
    records: list[tuple[int, float, float, float, float, float, float]] = []
    for row in imu_rows:
        try:
            ts_ns = ms_to_ns(int(float(row["unix_time"])))
            rec = (
                ts_ns,
                float(row.get("gyro_x", 0.0) or 0.0),
                float(row.get("gyro_y", 0.0) or 0.0),
                float(row.get("gyro_z", 0.0) or 0.0),
                float(row.get("acc_x", 0.0) or 0.0),
                float(row.get("acc_y", 0.0) or 0.0),
                float(row.get("acc_z", 0.0) or 0.0),
            )
        except (ValueError, KeyError):
            continue
        records.append(rec)

    records.sort(key=lambda x: x[0])

    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "#timestamp [ns]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
            ]
        )
        for rec in records:
            w.writerow(rec)

    if len(records) < 2:
        return len(records), 0.0

    dts = [records[i + 1][0] - records[i][0] for i in range(len(records) - 1) if records[i + 1][0] > records[i][0]]
    if not dts:
        return len(records), 0.0
    median_dt_ns = sorted(dts)[len(dts) // 2]
    rate_hz = 1e9 / median_dt_ns
    return len(records), rate_hz


def infer_right_baseline(session_dir: Path) -> tuple[list[float], list[float]]:
    left_desc = load_csv(session_dir / "left_depth_descriptors.csv")
    right_desc = load_csv(session_dir / "right_depth_descriptors.csv")

    right_by_ts = {int(float(r["timestamp_ms"])): r for r in right_desc if "timestamp_ms" in r}

    vecs = []
    for l in left_desc:
        try:
            ts = int(float(l["timestamp_ms"]))
        except (ValueError, KeyError):
            continue
        r = right_by_ts.get(ts)
        if not r:
            continue
        try:
            lx, ly, lz = (
                float(l["create_pose_location_x"]),
                float(l["create_pose_location_y"]),
                float(l["create_pose_location_z"]),
            )
            rx, ry, rz = (
                float(r["create_pose_location_x"]),
                float(r["create_pose_location_y"]),
                float(r["create_pose_location_z"]),
            )
        except (ValueError, KeyError):
            continue
        vecs.append((rx - lx, ry - ly, rz - lz))

    if not vecs:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    arr = np.array(vecs, dtype=float)
    med = np.median(arr, axis=0).tolist()
    std = np.std(arr, axis=0).tolist()
    return med, std


def write_cam_yaml(path: Path, name: str, intr: dict[str, float], width: int, height: int, t_bs: list[float]) -> None:
    yaml = f"""sensor_type: camera
+comment: {name}
+T_BS:
+  rows: 4
+  cols: 4
+  data: [1.0, 0.0, 0.0, {t_bs[0]:.9f}, 0.0, 1.0, 0.0, {t_bs[1]:.9f}, 0.0, 0.0, 1.0, {t_bs[2]:.9f}, 0.0, 0.0, 0.0, 1.0]
+rate_hz: 72
+resolution: [{width}, {height}]
+camera_model: pinhole
+intrinsics: [{intr['fx']:.9f}, {intr['fy']:.9f}, {intr['cx']:.9f}, {intr['cy']:.9f}]
+distortion_model: radtan
+distortion_coefficients: [0.0, 0.0, 0.0, 0.0]
+"""
    # Remove helper plus-prefix used to keep f-string readable.
    path.write_text("\n".join(line[1:] if line.startswith("+") else line for line in yaml.splitlines()) + "\n")


def write_imu_yaml(path: Path, rate_hz: float) -> None:
    yaml = f"""sensor_type: imu
+comment: Quest3 head IMU (gyro+accel) exported from imu.csv
+T_BS:
+  rows: 4
+  cols: 4
+  data: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
+rate_hz: {rate_hz:.3f}
+gyroscope_noise_density: 0.001
+gyroscope_random_walk: 0.0001
+accelerometer_noise_density: 0.01
+accelerometer_random_walk: 0.001
+"""
    path.write_text("\n".join(line[1:] if line.startswith("+") else line for line in yaml.splitlines()) + "\n")


def write_hmd_tum(path: Path, rows: list[dict[str, str]]) -> int:
    written = 0
    with path.open("w") as f:
        for r in rows:
            try:
                t_s = float(r["unix_time"]) / 1000.0
                px = float(r["pos_x"])
                py = float(r["pos_y"])
                pz = float(r["pos_z"])
                qx = float(r["rot_x"])
                qy = float(r["rot_y"])
                qz = float(r["rot_z"])
                qw = float(r["rot_w"])
            except (ValueError, KeyError):
                continue
            f.write(f"{t_s:.9f} {px:.9f} {py:.9f} {pz:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
            written += 1
    return written


def export_depth(session_dir: Path, out_mav0: Path, side: str) -> int:
    desc_rows = load_csv(session_dir / f"{side}_depth_descriptors.csv")
    by_ts = {}
    for r in desc_rows:
        try:
            by_ts[int(float(r["timestamp_ms"]))] = (int(float(r["width"])), int(float(r["height"])))
        except (ValueError, KeyError):
            continue

    in_dir = session_dir / f"{side}_depth"
    out_dir = out_mav0 / ("depth0" if side == "left" else "depth1") / "data"
    ensure_dir(out_dir)

    rows: list[tuple[int, str]] = []
    written = 0
    for raw_path in sorted(in_dir.glob("*.raw")):
        try:
            ts_ms = int(raw_path.stem)
        except ValueError:
            continue
        if ts_ms not in by_ts:
            continue
        w, h = by_ts[ts_ms]
        depth = np.fromfile(raw_path, dtype=np.uint16)
        if depth.size < w * h:
            continue
        depth = depth[: w * h].reshape((h, w))
        ts_ns = ms_to_ns(ts_ms)
        name = f"{ts_ns}.png"
        if not cv2.imwrite(str(out_dir / name), depth):
            continue
        rows.append((ts_ns, name))
        written += 1

    write_cam_csv(out_dir.parent / "data.csv", rows)
    return written


def main() -> int:
    args = parse_args()
    session_dir = Path(args.session_dir).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()

    if not session_dir.exists():
        raise FileNotFoundError(f"Session dir not found: {session_dir}")

    left_fmt = load_json(session_dir / "left_camera_image_format.json")
    right_fmt = load_json(session_dir / "right_camera_image_format.json")
    left_chars = load_json(session_dir / "left_camera_characteristics.json")
    right_chars = load_json(session_dir / "right_camera_characteristics.json")

    imu_rows = load_csv(session_dir / "imu.csv")
    hmd_rows = load_csv(session_dir / "hmd_poses.csv")

    left_yuv = list_ts_files(session_dir / "left_camera_raw", "yuv")
    right_yuv = list_ts_files(session_dir / "right_camera_raw", "yuv")

    common_ts = sorted(set(left_yuv) & set(right_yuv))
    if args.max_frames > 0:
        common_ts = common_ts[: args.max_frames]

    if not common_ts:
        raise RuntimeError("No matched stereo timestamps found")

    out_mav0 = out_dir / "mav0"
    cam0_data = out_mav0 / "cam0" / "data"
    cam1_data = out_mav0 / "cam1" / "data"
    imu0_dir = out_mav0 / "imu0"
    gt_dir = out_mav0 / "state_groundtruth_estimate0"
    meta_dir = out_mav0 / "meta"

    for p in [cam0_data, cam1_data, imu0_dir, gt_dir, meta_dir]:
        ensure_dir(p)

    cam0_rows: list[tuple[int, str]] = []
    cam1_rows: list[tuple[int, str]] = []

    for i, ts_ms in enumerate(common_ts):
        y0 = read_y_plane(left_yuv[ts_ms], left_fmt)
        y1 = read_y_plane(right_yuv[ts_ms], right_fmt)

        ts_ns = ms_to_ns(ts_ms)
        name = f"{ts_ns}.png"

        ok0 = cv2.imwrite(str(cam0_data / name), y0)
        ok1 = cv2.imwrite(str(cam1_data / name), y1)
        if not (ok0 and ok1):
            continue

        cam0_rows.append((ts_ns, name))
        cam1_rows.append((ts_ns, name))

        if (i + 1) % 100 == 0:
            print(f"exported stereo frames: {i + 1}/{len(common_ts)}")

    write_cam_csv(out_mav0 / "cam0" / "data.csv", cam0_rows)
    write_cam_csv(out_mav0 / "cam1" / "data.csv", cam1_rows)
    write_orbslam_times(meta_dir / "orbslam_times.txt", [ts for ts, _ in cam0_rows])

    imu_count, imu_rate_hz = write_imu_csv(imu0_dir / "data.csv", imu_rows)
    hmd_count = write_hmd_tum(gt_dir / "tum.txt", hmd_rows)

    l_intr = left_chars["intrinsics"]
    r_intr = right_chars["intrinsics"]
    l_res = left_chars["sensor"]["pixelArraySize"]
    r_res = right_chars["sensor"]["pixelArraySize"]

    right_baseline_med, right_baseline_std = infer_right_baseline(session_dir)

    write_cam_yaml(
        out_mav0 / "cam0" / "sensor.yaml",
        "Quest3 left camera",
        l_intr,
        int(l_res["width"]),
        int(l_res["height"]),
        [0.0, 0.0, 0.0],
    )
    write_cam_yaml(
        out_mav0 / "cam1" / "sensor.yaml",
        "Quest3 right camera",
        r_intr,
        int(r_res["width"]),
        int(r_res["height"]),
        [
            float(right_baseline_med[0]),
            float(right_baseline_med[1]),
            float(right_baseline_med[2]),
        ],
    )
    write_imu_yaml(out_mav0 / "imu0" / "sensor.yaml", imu_rate_hz)

    depth_written = {"left": 0, "right": 0}
    if args.include_depth:
        depth_written["left"] = export_depth(session_dir, out_mav0, "left")
        depth_written["right"] = export_depth(session_dir, out_mav0, "right")

    summary = {
        "session_dir": str(session_dir),
        "stereo_total_left": len(left_yuv),
        "stereo_total_right": len(right_yuv),
        "stereo_common": len(common_ts),
        "stereo_exported": len(cam0_rows),
        "imu_samples": imu_count,
        "imu_rate_hz_median": imu_rate_hz,
        "hmd_rows": hmd_count,
        "depth_exported": depth_written,
        "right_minus_left_baseline_median_m": right_baseline_med,
        "right_minus_left_baseline_std_m": right_baseline_std,
        "baseline_norm_m": math.sqrt(sum(float(v) * float(v) for v in right_baseline_med)),
    }

    (meta_dir / "session_summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    print("Export complete")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
