from mcap.writer import Writer
from mcap_protobuf.writer import ProtobufWriter
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import numpy as np, cv2, pandas as pd, glob, json

writer = Writer(open("quest3.mcap", "wb"))
pb = ProtobufWriter(writer)

def ms_to_ns(ms): return int(ms) * 1_000_000

# --- RGB YUV -> Image ---
def write_yuv(folder, topic, width, height, frame_id):
    for f in sorted(glob.glob(folder+"/*.yuv")):
        ts = ms_to_ns(f.split("/")[-1].split(".")[0])
        yuv = np.fromfile(f, np.uint8).reshape((height*3//2, width))
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

        msg = Image()
        msg.header.stamp.nanosec = ts
        msg.header.frame_id = frame_id
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()

        pb.write_message(topic, msg, ts)

# --- Pose CSV -> PoseStamped ---
def write_pose(csv_path, topic, frame_id):
    df = pd.read_csv(csv_path)
    for _, r in df.iterrows():
        ts = ms_to_ns(r["timestamp"])
        msg = PoseStamped()
        msg.header.stamp.nanosec = ts
        msg.header.frame_id = frame_id
        msg.pose.position.x = r["px"]
        msg.pose.position.y = r["py"]
        msg.pose.position.z = r["pz"]
        msg.pose.orientation.x = r["qx"]
        msg.pose.orientation.y = r["qy"]
        msg.pose.orientation.z = r["qz"]
        msg.pose.orientation.w = r["qw"]

        pb.write_message(topic, msg, ts)


def write_depth(folder, topic, width, height, frame_id):
    for f in sorted(glob.glob(folder+"/*.raw")):
        ts = ms_to_ns(f.split("/")[-1].split(".")[0])
        depth = np.fromfile(f, np.uint16).reshape((height, width))

        msg = Image()
        msg.header.stamp.nanosec = ts
        msg.header.frame_id = frame_id
        msg.height, msg.width = depth.shape
        msg.encoding = "16UC1"
        msg.step = msg.width * 2
        msg.data = depth.tobytes()

        pb.write_message(topic, msg, ts)

def write_camerainfo(json_path, topic, frame_id, ts):
    j = json.load(open(json_path))
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.height = j["height"]
    msg.width = j["width"]
    msg.k = j["K"]
    msg.d = j["D"]
    msg.r = j["R"]
    msg.p = j["P"]
    pb.write_message(topic, msg, ts)


# EXAMPLES
write_yuv("left_camera_raw", "/camera/left/image_raw", 1280, 720, "left_camera_frame")
write_pose("hmd_poses.csv", "/hmd/pose", "hmd_frame")
write_depth("left_depth", "/depth/left/image_raw", 320, 240, "left_depth_frame")
write_camerainfo("left_camera_characteristics.json",
                 "/camera/left/camera_info",
                 "left_camera_frame",
                 0)

writer.finish()
