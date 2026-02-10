"""Convert EuRoC-style dataset layout to ROS2-profile MCAP for Foxglove."""

from __future__ import annotations

import argparse
import ast
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
from mcap_ros2.writer import Writer


IMAGE_MSGDEF = """std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
===
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
===
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

CAMERA_INFO_MSGDEF = """std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi
===
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
===
MSG: sensor_msgs/RegionOfInterest
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify
===
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

IMU_MSGDEF = """std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
===
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
===
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
===
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
===
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

POSE_STAMPED_MSGDEF = """std_msgs/Header header
geometry_msgs/Pose pose
===
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
===
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
===
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
===
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
===
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


ImageMode = Literal["auto", "mono", "rgb"]


@dataclass
class CameraSensor:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: list[float]
    t_bs: list[float]


@dataclass
class EurocToMcapSummary:
    output: str
    stereo_frames_written: int
    stereo_frames_target: int
    imu_samples_written: int
    depth0_written: int
    depth1_written: int
    groundtruth_pose_written: int
    estimated_pose_written: int
    baseline_m_used_for_cam1_projection: float
    image_mode: ImageMode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert EuRoC export to ROS2 MCAP")
    parser.add_argument("--euroc-dir", required=True, help="Path containing mav0 directory")
    parser.add_argument("--out", required=True, help="Output MCAP file path")
    parser.add_argument("--estimated-tum", default="", help="Optional estimated trajectory in TUM format")
    parser.add_argument("--reference-tum", default="", help="Optional reference trajectory in TUM format")
    parser.add_argument("--include-depth", action="store_true", help="Include /depth0 and /depth1 image topics")
    parser.add_argument("--max-frames", type=int, default=0, help="Limit number of stereo frames (0 = all)")
    parser.add_argument(
        "--image-mode",
        choices=["auto", "mono", "rgb"],
        default="auto",
        help="Image encoding to publish. auto infers from PNG channels.",
    )
    return parser.parse_args()


def parse_sensor_yaml(path: Path) -> CameraSensor:
    data: dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        k, v = line.split(":", 1)
        data[k.strip()] = v.strip()

    resolution = ast.literal_eval(data["resolution"])
    intr = ast.literal_eval(data["intrinsics"])
    dist = ast.literal_eval(data.get("distortion_coefficients", "[0,0,0,0]"))

    t_bs = [0.0, 0.0, 0.0]
    t_line = data.get("data")
    if t_line:
        flat = ast.literal_eval(t_line)
        if isinstance(flat, list) and len(flat) == 16:
            t_bs = [float(flat[3]), float(flat[7]), float(flat[11])]

    return CameraSensor(
        width=int(resolution[0]),
        height=int(resolution[1]),
        fx=float(intr[0]),
        fy=float(intr[1]),
        cx=float(intr[2]),
        cy=float(intr[3]),
        dist=[float(x) for x in dist],
        t_bs=t_bs,
    )


def read_data_csv(path: Path) -> list[tuple[int, str]]:
    rows: list[tuple[int, str]] = []
    with path.open(newline="") as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
            if len(row) < 2:
                continue
            try:
                rows.append((int(row[0]), row[1]))
            except ValueError:
                continue
    return rows


def ns_to_stamp(ns: int) -> dict[str, int]:
    return {"sec": int(ns // 1_000_000_000), "nanosec": int(ns % 1_000_000_000)}


def make_header(ns: int, frame_id: str) -> dict:
    return {"stamp": ns_to_stamp(ns), "frame_id": frame_id}


def read_tum(path: Path) -> list[tuple[int, dict]]:
    out: list[tuple[int, dict]] = []
    if not path.exists():
        return out
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 8:
            continue
        try:
            ts_ns = int(float(parts[0]) * 1e9)
            px, py, pz = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])
        except ValueError:
            continue
        out.append(
            (
                ts_ns,
                {
                    "header": make_header(ts_ns, "world"),
                    "pose": {
                        "position": {"x": px, "y": py, "z": pz},
                        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                    },
                },
            )
        )
    return out


def camera_info_msg(cam: CameraSensor, ns: int, frame_id: str, projection_tx: float = 0.0) -> dict:
    d = list(cam.dist) + [0.0] * (5 - len(cam.dist))
    d = d[:5]
    k = [cam.fx, 0.0, cam.cx, 0.0, cam.fy, cam.cy, 0.0, 0.0, 1.0]
    r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    p = [cam.fx, 0.0, cam.cx, projection_tx, 0.0, cam.fy, cam.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return {
        "header": make_header(ns, frame_id),
        "height": cam.height,
        "width": cam.width,
        "distortion_model": "plumb_bob",
        "d": d,
        "k": k,
        "r": r,
        "p": p,
        "binning_x": 0,
        "binning_y": 0,
        "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
    }


def _prepare_image_payload(image: np.ndarray, image_mode: ImageMode) -> tuple[np.ndarray, str, int]:
    if image_mode == "mono":
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image, "mono8", int(image.shape[1])

    if image_mode == "rgb":
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image, "bgr8", int(image.shape[1] * 3)

    # auto
    if image.ndim == 2:
        return image, "mono8", int(image.shape[1])
    return image, "bgr8", int(image.shape[1] * 3)


def write_stereo_images(
    writer: Writer,
    image_schema,
    cam0_rows: list[tuple[int, str]],
    cam1_rows: list[tuple[int, str]],
    mav0: Path,
    max_frames: int,
    image_mode: ImageMode,
) -> tuple[int, int, int, int]:
    cam0_by_ts = {ts: fn for ts, fn in cam0_rows}
    cam1_by_ts = {ts: fn for ts, fn in cam1_rows}
    common_ts = sorted(set(cam0_by_ts) & set(cam1_by_ts))
    if max_frames > 0:
        common_ts = common_ts[:max_frames]

    if not common_ts:
        return 0, 0, 0, 0

    written = 0
    first_ts, last_ts = common_ts[0], common_ts[-1]

    for idx, ts_ns in enumerate(common_ts, start=1):
        p0 = mav0 / "cam0" / "data" / cam0_by_ts[ts_ns]
        p1 = mav0 / "cam1" / "data" / cam1_by_ts[ts_ns]
        im0 = cv2.imread(str(p0), cv2.IMREAD_UNCHANGED)
        im1 = cv2.imread(str(p1), cv2.IMREAD_UNCHANGED)
        if im0 is None or im1 is None:
            continue

        im0, enc0, step0 = _prepare_image_payload(im0, image_mode)
        im1, enc1, step1 = _prepare_image_payload(im1, image_mode)

        m0 = {
            "header": make_header(ts_ns, "cam0"),
            "height": int(im0.shape[0]),
            "width": int(im0.shape[1]),
            "encoding": enc0,
            "is_bigendian": 0,
            "step": step0,
            "data": im0.tobytes(),
        }
        m1 = {
            "header": make_header(ts_ns, "cam1"),
            "height": int(im1.shape[0]),
            "width": int(im1.shape[1]),
            "encoding": enc1,
            "is_bigendian": 0,
            "step": step1,
            "data": im1.tobytes(),
        }
        writer.write_message("/cam0/image_raw", image_schema, m0, log_time=ts_ns, publish_time=ts_ns)
        writer.write_message("/cam1/image_raw", image_schema, m1, log_time=ts_ns, publish_time=ts_ns)
        written += 1

        if idx % 100 == 0:
            print(f"wrote stereo frames: {idx}/{len(common_ts)}")

    return written, len(common_ts), first_ts, last_ts


def write_imu(writer: Writer, imu_schema, imu_csv: Path, min_ts: int, max_ts: int) -> int:
    count = 0
    with imu_csv.open(newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                ts_ns = int(row["#timestamp [ns]"])
                gx = float(row["w_RS_S_x [rad s^-1]"])
                gy = float(row["w_RS_S_y [rad s^-1]"])
                gz = float(row["w_RS_S_z [rad s^-1]"])
                ax = float(row["a_RS_S_x [m s^-2]"])
                ay = float(row["a_RS_S_y [m s^-2]"])
                az = float(row["a_RS_S_z [m s^-2]"])
            except (KeyError, ValueError):
                continue
            if min_ts and max_ts and not (min_ts <= ts_ns <= max_ts):
                continue
            msg = {
                "header": make_header(ts_ns, "imu0"),
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "orientation_covariance": [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "angular_velocity": {"x": gx, "y": gy, "z": gz},
                "angular_velocity_covariance": [0.0] * 9,
                "linear_acceleration": {"x": ax, "y": ay, "z": az},
                "linear_acceleration_covariance": [0.0] * 9,
            }
            writer.write_message("/imu0", imu_schema, msg, log_time=ts_ns, publish_time=ts_ns)
            count += 1
    return count


def write_depth_images(
    writer: Writer,
    image_schema,
    depth_rows: list[tuple[int, str]],
    depth_dir: Path,
    topic: str,
    frame_id: str,
    min_ts: int,
    max_ts: int,
) -> int:
    count = 0
    for ts_ns, name in depth_rows:
        if min_ts and max_ts and not (min_ts <= ts_ns <= max_ts):
            continue
        depth = cv2.imread(str(depth_dir / name), cv2.IMREAD_UNCHANGED)
        if depth is None:
            continue
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        msg = {
            "header": make_header(ts_ns, frame_id),
            "height": int(depth.shape[0]),
            "width": int(depth.shape[1]),
            "encoding": "16UC1",
            "is_bigendian": 0,
            "step": int(depth.shape[1] * 2),
            "data": depth.tobytes(),
        }
        writer.write_message(topic, image_schema, msg, log_time=ts_ns, publish_time=ts_ns)
        count += 1
    return count


def write_poses(writer: Writer, pose_schema, poses: list[tuple[int, dict]], topic: str, min_ts: int, max_ts: int) -> int:
    count = 0
    for ts_ns, msg in poses:
        if min_ts and max_ts and not (min_ts <= ts_ns <= max_ts):
            continue
        writer.write_message(topic, pose_schema, msg, log_time=ts_ns, publish_time=ts_ns)
        count += 1
    return count


def convert_euroc_to_mcap(
    euroc_dir: Path,
    out_path: Path,
    *,
    estimated_tum: Path | None = None,
    reference_tum: Path | None = None,
    include_depth: bool = False,
    max_frames: int = 0,
    image_mode: ImageMode = "auto",
) -> EurocToMcapSummary:
    euroc_dir = euroc_dir.expanduser().resolve()
    mav0 = euroc_dir / "mav0" if (euroc_dir / "mav0").exists() else euroc_dir

    out_path = out_path.expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cam0_rows = read_data_csv(mav0 / "cam0" / "data.csv")
    cam1_rows = read_data_csv(mav0 / "cam1" / "data.csv")
    if not cam0_rows or not cam1_rows:
        raise RuntimeError(f"Missing camera data.csv under {mav0}")

    cam0_sensor = parse_sensor_yaml(mav0 / "cam0" / "sensor.yaml")
    cam1_sensor = parse_sensor_yaml(mav0 / "cam1" / "sensor.yaml")
    baseline_m = float(np.linalg.norm(np.array(cam1_sensor.t_bs, dtype=float)))

    ref_poses = read_tum(reference_tum) if reference_tum and reference_tum.exists() else []
    est_poses = read_tum(estimated_tum) if estimated_tum and estimated_tum.exists() else []

    with Writer(str(out_path)) as writer:
        image_schema = writer.register_msgdef("sensor_msgs/msg/Image", IMAGE_MSGDEF)
        camera_info_schema = writer.register_msgdef("sensor_msgs/msg/CameraInfo", CAMERA_INFO_MSGDEF)
        imu_schema = writer.register_msgdef("sensor_msgs/msg/Imu", IMU_MSGDEF)
        pose_schema = writer.register_msgdef("geometry_msgs/msg/PoseStamped", POSE_STAMPED_MSGDEF)

        written_stereo, target_frames, min_ts, max_ts = write_stereo_images(
            writer,
            image_schema,
            cam0_rows,
            cam1_rows,
            mav0,
            max_frames,
            image_mode,
        )
        if written_stereo == 0:
            raise RuntimeError("No stereo images written")

        cam0_info = camera_info_msg(cam0_sensor, min_ts, "cam0", projection_tx=0.0)
        cam1_info = camera_info_msg(cam1_sensor, min_ts, "cam1", projection_tx=-(cam1_sensor.fx * baseline_m))
        writer.write_message("/cam0/camera_info", camera_info_schema, cam0_info, log_time=min_ts, publish_time=min_ts)
        writer.write_message("/cam1/camera_info", camera_info_schema, cam1_info, log_time=min_ts, publish_time=min_ts)

        imu_count = write_imu(writer, imu_schema, mav0 / "imu0" / "data.csv", min_ts, max_ts)

        depth0_count = 0
        depth1_count = 0
        if include_depth and (mav0 / "depth0" / "data.csv").exists() and (mav0 / "depth1" / "data.csv").exists():
            depth0_count = write_depth_images(
                writer,
                image_schema,
                read_data_csv(mav0 / "depth0" / "data.csv"),
                mav0 / "depth0" / "data",
                "/depth0/image_raw",
                "depth0",
                min_ts,
                max_ts,
            )
            depth1_count = write_depth_images(
                writer,
                image_schema,
                read_data_csv(mav0 / "depth1" / "data.csv"),
                mav0 / "depth1" / "data",
                "/depth1/image_raw",
                "depth1",
                min_ts,
                max_ts,
            )

        ref_count = write_poses(writer, pose_schema, ref_poses, "/groundtruth/pose", min_ts, max_ts)
        est_count = write_poses(writer, pose_schema, est_poses, "/slam/estimated_pose", min_ts, max_ts)

    return EurocToMcapSummary(
        output=str(out_path),
        stereo_frames_written=written_stereo,
        stereo_frames_target=target_frames,
        imu_samples_written=imu_count,
        depth0_written=depth0_count,
        depth1_written=depth1_count,
        groundtruth_pose_written=ref_count,
        estimated_pose_written=est_count,
        baseline_m_used_for_cam1_projection=baseline_m,
        image_mode=image_mode,
    )


def add_parser(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    parser = subparsers.add_parser("euroc-to-mcap", help="Convert EuRoC export into ROS2-profile MCAP")
    parser.add_argument("--euroc-dir", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--estimated-tum", default="")
    parser.add_argument("--reference-tum", default="")
    parser.add_argument("--include-depth", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--image-mode", choices=["auto", "mono", "rgb"], default="auto")
    return parser


def run_cli(args: argparse.Namespace) -> int:
    summary = convert_euroc_to_mcap(
        Path(args.euroc_dir),
        Path(args.out),
        estimated_tum=Path(args.estimated_tum).expanduser().resolve() if args.estimated_tum else None,
        reference_tum=Path(args.reference_tum).expanduser().resolve() if args.reference_tum else None,
        include_depth=args.include_depth,
        max_frames=args.max_frames,
        image_mode=args.image_mode,
    )
    print("EuRoC -> MCAP conversion complete")
    print(f"output: {summary.output}")
    print(f"stereo_frames_written: {summary.stereo_frames_written}/{summary.stereo_frames_target}")
    print(f"imu_samples_written: {summary.imu_samples_written}")
    print(f"depth0_written: {summary.depth0_written}")
    print(f"depth1_written: {summary.depth1_written}")
    print(f"groundtruth_pose_written: {summary.groundtruth_pose_written}")
    print(f"estimated_pose_written: {summary.estimated_pose_written}")
    print(f"baseline_m_used_for_cam1_projection: {summary.baseline_m_used_for_cam1_projection:.6f}")
    return 0


def main() -> int:
    args = parse_args()
    return run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
