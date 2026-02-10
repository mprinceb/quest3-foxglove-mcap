"""Open3D RGB-D odometry baseline for Quest EuRoC exports."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


def _import_open3d():
    try:
        import open3d as o3d  # type: ignore
    except Exception as exc:  # pragma: no cover - import guard
        raise RuntimeError(
            "Open3D is not installed for this Python environment. "
            "Run with: uv run --with open3d python -m slam.cli run-open3d-rgbd ..."
        ) from exc
    return o3d


@dataclass
class Open3DSummary:
    frames: int
    successful_steps: int
    skipped_no_depth: int
    skipped_odometry_failure: int
    trajectory: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Open3D RGB-D odometry on EuRoC export")
    parser.add_argument("--euroc-dir", required=True, help="Path with mav0/ subfolder")
    parser.add_argument("--out-tum", required=True)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--max-depth-gap-ms", type=int, default=40)
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--depth-trunc", type=float, default=15.0)
    parser.add_argument("--method", choices=["hybrid", "color"], default="hybrid")
    return parser.parse_args()


def read_data_csv(path: Path) -> list[tuple[int, str]]:
    rows: list[tuple[int, str]] = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                rows.append((int(row[0]), row[1]))
            except ValueError:
                continue
    return rows


def parse_sensor_yaml(path: Path) -> tuple[float, float, float, float]:
    vals = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        vals[key.strip()] = value.strip()

    intr = vals.get("intrinsics")
    if not intr:
        raise RuntimeError(f"intrinsics missing in {path}")
    fx, fy, cx, cy = [float(x.strip()) for x in intr.strip("[]").split(",")]
    return fx, fy, cx, cy


def rot_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
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


def pose_to_tum_line(ts_ns: int, t_wc: np.ndarray, r_wc: np.ndarray) -> str:
    qx, qy, qz, qw = rot_to_quat(r_wc)
    ts_s = ts_ns / 1e9
    return (
        f"{ts_s:.9f} {t_wc[0,0]:.9f} {t_wc[1,0]:.9f} {t_wc[2,0]:.9f} "
        f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}"
    )


def _nearest_ts(target: int, ts_sorted: list[int], max_gap_ns: int) -> int | None:
    if not ts_sorted:
        return None
    idx = int(np.searchsorted(ts_sorted, target))
    cands = []
    if idx < len(ts_sorted):
        cands.append(ts_sorted[idx])
    if idx > 0:
        cands.append(ts_sorted[idx - 1])
    if not cands:
        return None
    best = min(cands, key=lambda t: abs(t - target))
    if abs(best - target) > max_gap_ns:
        return None
    return best


def run_open3d_rgbd_odometry(
    euroc_dir: Path,
    out_tum: Path,
    *,
    max_frames: int = 0,
    max_depth_gap_ms: int = 40,
    depth_scale: float = 1000.0,
    depth_trunc: float = 15.0,
    method: str = "hybrid",
) -> Open3DSummary:
    o3d = _import_open3d()

    euroc_dir = euroc_dir.expanduser().resolve()
    mav0 = euroc_dir / "mav0" if (euroc_dir / "mav0").exists() else euroc_dir
    out_tum = out_tum.expanduser().resolve()
    out_tum.parent.mkdir(parents=True, exist_ok=True)

    cam_rows = read_data_csv(mav0 / "cam0" / "data.csv")
    depth_rows = read_data_csv(mav0 / "depth0" / "data.csv")
    if not cam_rows or not depth_rows:
        raise RuntimeError("Open3D odometry requires cam0 and depth0 data.csv")

    if max_frames > 0:
        cam_rows = cam_rows[:max_frames]

    fx, fy, cx, cy = parse_sensor_yaml(mav0 / "cam0" / "sensor.yaml")
    intrinsic = o3d.camera.PinholeCameraIntrinsic()

    depth_by_ts = {ts: name for ts, name in depth_rows}
    depth_ts_sorted = sorted(depth_by_ts.keys())
    max_gap_ns = max_depth_gap_ms * 1_000_000

    def load_rgbd(ts_ns: int, image_name: str):
        depth_ts = _nearest_ts(ts_ns, depth_ts_sorted, max_gap_ns)
        if depth_ts is None:
            return None

        image = cv2.imread(str(mav0 / "cam0" / "data" / image_name), cv2.IMREAD_UNCHANGED)
        depth = cv2.imread(str(mav0 / "depth0" / "data" / depth_by_ts[depth_ts]), cv2.IMREAD_UNCHANGED)
        if image is None or depth is None:
            return None

        if image.ndim == 2:
            color = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        else:
            color = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)

        if depth.shape[:2] != color.shape[:2]:
            depth = cv2.resize(depth, (color.shape[1], color.shape[0]), interpolation=cv2.INTER_NEAREST)

        intrinsic.set_intrinsics(width=color.shape[1], height=color.shape[0], fx=fx, fy=fy, cx=cx, cy=cy)

        o3d_color = o3d.geometry.Image(color)
        o3d_depth = o3d.geometry.Image(depth)
        return o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color,
            o3d_depth,
            depth_scale=depth_scale,
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )

    r_cw = np.eye(3, dtype=np.float64)
    t_cw = np.zeros((3, 1), dtype=np.float64)
    r_wc = r_cw.T
    t_wc = -r_wc @ t_cw

    first_ts, first_name = cam_rows[0]
    prev_rgbd = load_rgbd(first_ts, first_name)
    lines = [pose_to_tum_line(first_ts, t_wc, r_wc)]

    success = 0
    skipped_no_depth = 0
    skipped_fail = 0

    for ts_ns, image_name in cam_rows[1:]:
        curr_rgbd = load_rgbd(ts_ns, image_name)
        if prev_rgbd is None or curr_rgbd is None:
            skipped_no_depth += 1
            prev_rgbd = curr_rgbd
            lines.append(pose_to_tum_line(ts_ns, t_wc, r_wc))
            continue

        if method == "hybrid":
            jacobian = o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm()
        else:
            jacobian = o3d.pipelines.odometry.RGBDOdometryJacobianFromColorTerm()

        ok, trans, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
            source=curr_rgbd,
            target=prev_rgbd,
            pinhole_camera_intrinsic=intrinsic,
            odo_init=np.eye(4),
            jacobian=jacobian,
        )

        if not ok:
            skipped_fail += 1
            prev_rgbd = curr_rgbd
            lines.append(pose_to_tum_line(ts_ns, t_wc, r_wc))
            continue

        r_cp = np.asarray(trans[:3, :3], dtype=np.float64)
        t_cp = np.asarray(trans[:3, 3], dtype=np.float64).reshape(3, 1)

        r_cw = r_cp @ r_cw
        t_cw = r_cp @ t_cw + t_cp
        r_wc = r_cw.T
        t_wc = -r_wc @ t_cw

        success += 1
        prev_rgbd = curr_rgbd
        lines.append(pose_to_tum_line(ts_ns, t_wc, r_wc))

    out_tum.write_text("\n".join(lines) + "\n")
    return Open3DSummary(
        frames=len(cam_rows),
        successful_steps=success,
        skipped_no_depth=skipped_no_depth,
        skipped_odometry_failure=skipped_fail,
        trajectory=str(out_tum),
    )


def add_parser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser = subparsers.add_parser("run-open3d-rgbd", help="Run Open3D RGB-D odometry on EuRoC export")
    parser.add_argument("--euroc-dir", required=True)
    parser.add_argument("--out-tum", required=True)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--max-depth-gap-ms", type=int, default=40)
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--depth-trunc", type=float, default=15.0)
    parser.add_argument("--method", choices=["hybrid", "color"], default="hybrid")
    return parser


def run_cli(args: argparse.Namespace) -> int:
    summary = run_open3d_rgbd_odometry(
        Path(args.euroc_dir),
        Path(args.out_tum),
        max_frames=args.max_frames,
        max_depth_gap_ms=args.max_depth_gap_ms,
        depth_scale=args.depth_scale,
        depth_trunc=args.depth_trunc,
        method=args.method,
    )
    print("Open3D RGB-D odometry complete")
    print(
        f"frames={summary.frames} successful_steps={summary.successful_steps} "
        f"skipped_no_depth={summary.skipped_no_depth} skipped_odometry_failure={summary.skipped_odometry_failure}"
    )
    print(f"trajectory={summary.trajectory}")
    return 0


def main() -> int:
    args = parse_args()
    return run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
