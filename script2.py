from mcap_protobuf.writer import Writer
from google.protobuf.timestamp_pb2 import Timestamp

f = open("quest3.mcap", "wb")
writer = Writer(f)


import pandas as pd
from foxglove_schemas_protobuf.Vector3_pb2 import Vector3
from foxglove_schemas_protobuf.Quaternion_pb2 import Quaternion
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform

def ms_to_ns(ms): return int(ms) * 1_000_000

def set_timestamp(msg, ns):
    msg.timestamp.CopyFrom(
        Timestamp(
            seconds=int(ns // 1_000_000_000),
            nanos=int(ns % 1_000_000_000),
        )
    )

df = pd.read_csv("hmd_poses.csv")

for _, r in df.iterrows():
    ts = ms_to_ns(r["unix_time"])

    msg = FrameTransform(
        parent_frame_id="world",
        child_frame_id="hmd",
        translation=Vector3(x=r["pos_x"], y=r["pos_y"], z=r["pos_z"]),
        rotation=Quaternion(
            x=r["rot_x"], y=r["rot_y"], z=r["rot_z"], w=r["rot_w"]
        ),
    )
    set_timestamp(msg, ts)
    writer.write_message("/tf", msg, log_time=ts, publish_time=ts)


import glob, cv2, numpy as np
import os
import json
from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage


def first_timestamp_ns(folder, ext):
    paths = sorted(glob.glob(os.path.join(folder, f"*.{ext}")))
    if not paths:
        return 0
    ts_ms = int(os.path.basename(paths[0]).split(".")[0])
    return ms_to_ns(ts_ms)


def write_rgb(folder, raw_topic, compressed_topic, frame_id, image_format_json):
    fmt = json.load(open(image_format_json))
    width = int(fmt["width"])
    height = int(fmt["height"])
    y_row_stride = int(fmt["planes"][0]["rowStride"])
    uv_row_stride = int(fmt["planes"][1]["rowStride"])
    uv_pixel_stride = int(fmt["planes"][1]["pixelStride"])
    y_plane_size = int(fmt["planes"][0]["bufferSize"])
    u_plane_size = int(fmt["planes"][1]["bufferSize"])
    v_plane_size = int(fmt["planes"][2]["bufferSize"])

    y_size = y_plane_size
    uv_size = min(u_plane_size, v_plane_size)
    uv_expected = uv_row_stride * (height // 2)

    for fpath in sorted(glob.glob(folder + "/*.yuv")):
        ts = ms_to_ns(fpath.split("/")[-1].split(".")[0])

        data = np.fromfile(fpath, np.uint8)
        if data.size < y_size + 2 * uv_size:
            continue

        y = data[:y_size].reshape((height, y_row_stride))[:, :width]
        u_flat = data[y_size:y_size + uv_size]
        v_flat = data[y_size + uv_size:y_size + 2 * uv_size]
        if u_flat.size < uv_expected:
            u_flat = np.pad(u_flat, (0, uv_expected - u_flat.size), mode="edge")
        if v_flat.size < uv_expected:
            v_flat = np.pad(v_flat, (0, uv_expected - v_flat.size), mode="edge")
        u = u_flat.reshape((height // 2, uv_row_stride))
        v = v_flat.reshape((height // 2, uv_row_stride))

        if uv_pixel_stride == 2:
            u = u[:, ::2]
            v = v[:, ::2]
        else:
            u = u[:, :width // 2]
            v = v[:, :width // 2]

        i420 = np.concatenate((y.flatten(), u.flatten(), v.flatten()))
        i420 = i420.reshape((height * 3 // 2, width))
        bgr = cv2.cvtColor(i420, cv2.COLOR_YUV2BGR_I420)

        if raw_topic:
            msg = RawImage(
                frame_id=frame_id,
                width=width,
                height=height,
                encoding="bgr8",
                step=width * 3,
                data=bgr.tobytes(),
            )
            set_timestamp(msg, ts)
            writer.write_message(raw_topic, msg, log_time=ts, publish_time=ts)

        if compressed_topic:
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok:
                cmsg = CompressedImage(
                    frame_id=frame_id,
                    format="jpeg",
                    data=buf.tobytes(),
                )
                set_timestamp(cmsg, ts)
                writer.write_message(compressed_topic, cmsg, log_time=ts, publish_time=ts)




def write_depth(folder, topic, width):
    for fpath in sorted(glob.glob(folder + "/*.raw")):
        ts = ms_to_ns(fpath.split("/")[-1].split(".")[0])

        depth = np.fromfile(fpath, np.uint16)

        height = depth.size // width
        depth = depth[:height * width]
        depth = depth.reshape((height, width))

        msg = RawImage(
            frame_id=topic,
            width=width,
            height=height,
            encoding="16UC1",
            step=width * 2,
            data=depth.tobytes(),
        )
        set_timestamp(msg, ts)
        writer.write_message(topic, msg, log_time=ts, publish_time=ts)

from foxglove_schemas_protobuf.CameraCalibration_pb2 import CameraCalibration


def write_calib(json_path, topic, ts_ns):
    j = json.load(open(json_path))

    intr = j["intrinsics"]
    sensor = j["sensor"]["pixelArraySize"]

    width  = sensor["width"]
    height = sensor["height"]

    fx, fy = intr["fx"], intr["fy"]
    cx, cy = intr["cx"], intr["cy"]

    msg = CameraCalibration(
        frame_id=topic,
        width=width,
        height=height,
        distortion_model="plumb_bob",
        D=[0.0, 0.0, 0.0, 0.0, 0.0],  # Quest = no distortion
        K=[
            fx, 0,  cx,
            0,  fy, cy,
            0,  0,  1
        ],
        R=[1, 0, 0, 0, 1, 0, 0, 0, 1],
        P=[
            fx, 0,  cx, 0,
            0,  fy, cy, 0,
            0,  0,  1,  0
        ],
    )

    set_timestamp(msg, ts_ns)
    writer.write_message(topic, msg, log_time=ts_ns, publish_time=ts_ns)


write_rgb(
    "left_camera_raw",
    "/camera/left/image",
    "/cam_left/compressed",
    "camera_left",
    "left_camera_image_format.json",
)
write_rgb(
    "right_camera_raw",
    "/camera/right/image",
    "/cam_right/compressed",
    "camera_right",
    "right_camera_image_format.json",
)

# write_depth("left_depth", "/depth/left/image", 320)
# write_depth("right_depth", "/depth/right/image", 320)

left_calib_ts = first_timestamp_ns("left_camera_raw", "yuv")
right_calib_ts = first_timestamp_ns("right_camera_raw", "yuv")

write_calib("left_camera_characteristics.json", "/camera/left/calib", left_calib_ts)
write_calib("right_camera_characteristics.json", "/camera/right/calib", right_calib_ts)

writer.finish()
f.close()
