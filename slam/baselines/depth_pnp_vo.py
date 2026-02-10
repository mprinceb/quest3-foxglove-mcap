#!/usr/bin/env python3
"""Quest 3 baseline VO: ORB feature tracking + depth-assisted PnP.

This is a bootstrap estimator (not full SLAM):
- Tracks ORB features between consecutive left-camera frames.
- Uses nearest depth frame to lift previous-frame keypoints to 3D.
- Solves PnP RANSAC for relative motion.
- Accumulates pose and exports TUM trajectory.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class FrameData:
    ts_ms: int
    gray: np.ndarray
    depth: np.ndarray | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run depth-assisted ORB/PnP VO on Quest session")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--out-tum", required=True)
    parser.add_argument("--max-frames", type=int, default=0, help="Limit frames (0 = all)")
    parser.add_argument("--step", type=int, default=1, help="Process every Nth frame")
    parser.add_argument("--max-depth-gap-ms", type=int, default=35)
    parser.add_argument("--depth-scale", type=float, default=1000.0, help="Depth units per meter")
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=15.0)
    parser.add_argument("--min-inliers", type=int, default=35)
    parser.add_argument("--nfeatures", type=int, default=2000)
    return parser.parse_args()


def load_json(path: Path):
    with path.open() as f:
        return json.load(f)


def list_ts_files(folder: Path, suffix: str) -> dict[int, Path]:
    out = {}
    for p in folder.glob(f"*.{suffix}"):
        try:
            out[int(p.stem)] = p
        except ValueError:
            continue
    return out


def read_y_plane(yuv_path: Path, fmt: dict) -> np.ndarray:
    w = int(fmt["width"])
    h = int(fmt["height"])
    y_row_stride = int(fmt["planes"][0]["rowStride"])
    y_size = int(fmt["planes"][0]["bufferSize"])
    data = np.fromfile(yuv_path, dtype=np.uint8)
    if data.size < y_size:
        raise ValueError(f"YUV file too small: {yuv_path}")
    return data[:y_size].reshape((h, y_row_stride))[:, :w]


def nearest_depth_ts(ts_ms: int, depth_ts: list[int], max_gap_ms: int) -> int | None:
    if not depth_ts:
        return None
    idx = np.searchsorted(depth_ts, ts_ms)
    cands = []
    if idx < len(depth_ts):
        cands.append(depth_ts[idx])
    if idx > 0:
        cands.append(depth_ts[idx - 1])
    if not cands:
        return None
    best = min(cands, key=lambda x: abs(x - ts_ms))
    if abs(best - ts_ms) > max_gap_ms:
        return None
    return best


def load_depth(raw_path: Path, desc_row: dict[str, str]) -> np.ndarray | None:
    try:
        w = int(float(desc_row["width"]))
        h = int(float(desc_row["height"]))
    except (KeyError, ValueError):
        return None

    depth = np.fromfile(raw_path, dtype=np.uint16)
    if depth.size < w * h:
        return None
    return depth[: w * h].reshape((h, w))


def build_frame_stream(session_dir: Path, max_frames: int, step: int, max_depth_gap_ms: int) -> tuple[list[FrameData], dict[str, float]]:
    left_fmt = load_json(session_dir / "left_camera_image_format.json")
    intr = load_json(session_dir / "left_camera_characteristics.json")["intrinsics"]

    left_yuv = list_ts_files(session_dir / "left_camera_raw", "yuv")
    depth_raw = list_ts_files(session_dir / "left_depth", "raw")

    depth_desc_rows = {}
    with (session_dir / "left_depth_descriptors.csv").open(newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                depth_desc_rows[int(float(row["timestamp_ms"]))] = row
            except (KeyError, ValueError):
                continue

    depth_ts = sorted(set(depth_raw.keys()) & set(depth_desc_rows.keys()))

    frames: list[FrameData] = []
    all_ts = sorted(left_yuv.keys())
    if step > 1:
        all_ts = all_ts[::step]
    if max_frames > 0:
        all_ts = all_ts[:max_frames]

    for ts in all_ts:
        gray = read_y_plane(left_yuv[ts], left_fmt)
        dts = nearest_depth_ts(ts, depth_ts, max_depth_gap_ms)
        depth = None
        if dts is not None:
            depth = load_depth(depth_raw[dts], depth_desc_rows[dts])
        frames.append(FrameData(ts_ms=ts, gray=gray, depth=depth))

    k = {
        "fx": float(intr["fx"]),
        "fy": float(intr["fy"]),
        "cx": float(intr["cx"]),
        "cy": float(intr["cy"]),
        "img_w": float(int(left_fmt["width"])),
        "img_h": float(int(left_fmt["height"])),
    }
    return frames, k


def lift_points_to_3d(
    kp_prev,
    kp_curr,
    matches,
    depth_prev: np.ndarray,
    k: dict[str, float],
    depth_scale: float,
    min_depth: float,
    max_depth: float,
):
    depth_h, depth_w = depth_prev.shape
    pts_3d = []
    pts_2d = []

    sx = depth_w / k["img_w"]
    sy = depth_h / k["img_h"]

    for m in matches:
        u_prev, v_prev = kp_prev[m.queryIdx].pt
        u = int(round(u_prev * sx))
        v = int(round(v_prev * sy))
        if u < 0 or v < 0 or u >= depth_w or v >= depth_h:
            continue

        d_raw = int(depth_prev[v, u])
        if d_raw <= 0 or d_raw >= 65500:
            continue

        z = d_raw / depth_scale
        if z < min_depth or z > max_depth:
            continue

        x = (u_prev - k["cx"]) * z / k["fx"]
        y = (v_prev - k["cy"]) * z / k["fy"]
        pts_3d.append([x, y, z])

        u_curr, v_curr = kp_curr[m.trainIdx].pt
        pts_2d.append([u_curr, v_curr])

    if not pts_3d:
        return None, None
    return np.asarray(pts_3d, dtype=np.float32), np.asarray(pts_2d, dtype=np.float32)


def pose_to_tum_line(ts_ms: int, t_wc: np.ndarray, r_wc: np.ndarray) -> str:
    qx, qy, qz, qw = rot_to_quat(r_wc)
    ts_s = ts_ms / 1000.0
    return (
        f"{ts_s:.9f} {t_wc[0,0]:.9f} {t_wc[1,0]:.9f} {t_wc[2,0]:.9f} "
        f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}"
    )


def rot_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    # [qx, qy, qz, qw]
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def main() -> int:
    args = parse_args()
    session_dir = Path(args.session_dir).expanduser().resolve()
    out_tum = Path(args.out_tum).expanduser().resolve()
    out_tum.parent.mkdir(parents=True, exist_ok=True)

    frames, k = build_frame_stream(session_dir, args.max_frames, args.step, args.max_depth_gap_ms)
    if len(frames) < 2:
        raise RuntimeError("Not enough frames for VO")

    orb = cv2.ORB_create(nfeatures=args.nfeatures)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    k_mat = np.array([[k["fx"], 0, k["cx"]], [0, k["fy"], k["cy"]], [0, 0, 1]], dtype=np.float64)

    # Camera pose as camera-from-world (T_c_w).
    r_cw = np.eye(3, dtype=np.float64)
    t_cw = np.zeros((3, 1), dtype=np.float64)

    r_wc = r_cw.T
    t_wc = -r_wc @ t_cw

    traj_lines = [pose_to_tum_line(frames[0].ts_ms, t_wc, r_wc)]

    success_steps = 0
    skipped_no_depth = 0
    skipped_no_matches = 0
    skipped_no_pnp = 0

    prev = frames[0]
    kp_prev, des_prev = orb.detectAndCompute(prev.gray, None)

    for idx in range(1, len(frames)):
        curr = frames[idx]
        kp_curr, des_curr = orb.detectAndCompute(curr.gray, None)

        if prev.depth is None or des_prev is None or des_curr is None:
            skipped_no_depth += 1
            prev, kp_prev, des_prev = curr, kp_curr, des_curr
            traj_lines.append(pose_to_tum_line(curr.ts_ms, t_wc, r_wc))
            continue

        matches = bf.match(des_prev, des_curr)
        if len(matches) < args.min_inliers:
            skipped_no_matches += 1
            prev, kp_prev, des_prev = curr, kp_curr, des_curr
            traj_lines.append(pose_to_tum_line(curr.ts_ms, t_wc, r_wc))
            continue

        matches = sorted(matches, key=lambda m: m.distance)
        matches = matches[: min(len(matches), 500)]

        pts_3d, pts_2d = lift_points_to_3d(
            kp_prev,
            kp_curr,
            matches,
            prev.depth,
            k,
            args.depth_scale,
            args.min_depth,
            args.max_depth,
        )
        if pts_3d is None or len(pts_3d) < args.min_inliers:
            skipped_no_matches += 1
            prev, kp_prev, des_prev = curr, kp_curr, des_curr
            traj_lines.append(pose_to_tum_line(curr.ts_ms, t_wc, r_wc))
            continue

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts_3d,
            pts_2d,
            k_mat,
            None,
            iterationsCount=100,
            reprojectionError=4.0,
            confidence=0.999,
            flags=cv2.SOLVEPNP_EPNP,
        )

        if not ok or inliers is None or len(inliers) < args.min_inliers:
            skipped_no_pnp += 1
            prev, kp_prev, des_prev = curr, kp_curr, des_curr
            traj_lines.append(pose_to_tum_line(curr.ts_ms, t_wc, r_wc))
            continue

        r_cp, _ = cv2.Rodrigues(rvec)
        t_cp = tvec.reshape(3, 1)

        # PnP gives transform from previous camera frame -> current camera frame:
        # X_c_curr = R_cp * X_c_prev + t_cp
        # Compose in camera-from-world convention.
        r_cw = r_cp @ r_cw
        t_cw = r_cp @ t_cw + t_cp

        r_wc = r_cw.T
        t_wc = -r_wc @ t_cw

        success_steps += 1

        prev, kp_prev, des_prev = curr, kp_curr, des_curr
        traj_lines.append(pose_to_tum_line(curr.ts_ms, t_wc, r_wc))

    out_tum.write_text("\n".join(traj_lines) + "\n")

    print("Depth-PnP VO complete")
    print(f"frames={len(frames)} success_steps={success_steps}")
    print(
        f"skipped_no_depth={skipped_no_depth} "
        f"skipped_no_matches={skipped_no_matches} skipped_no_pnp={skipped_no_pnp}"
    )
    print(f"trajectory={out_tum}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
