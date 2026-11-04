#!/usr/bin/env python3
"""Save wrist-camera data at five poses chosen manually in RViz."""

import argparse
from collections import deque
import json
from pathlib import Path
import struct
import threading
import time
import zlib

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def stamp_ns(msg):
    t = msg.header.stamp
    return t.sec * 1_000_000_000 + t.nanosec


def vec3(v):
    return [float(v.x), float(v.y), float(v.z)]


def quat(v):
    return [float(v.x), float(v.y), float(v.z), float(v.w)]


def transform_point(tf, point):
    """Apply a ROS transform (target <- source) to an xyz point."""
    r = tf.transform.rotation
    x, y, z, w = r.x, r.y, r.z, r.w
    a, b, c = point
    tx, ty, tz = 2 * (y * c - z * b), 2 * (z * a - x * c), 2 * (x * b - y * a)
    t = tf.transform.translation
    return [a + w * tx + (y * tz - z * ty) + t.x,
            b + w * ty + (z * tx - x * tz) + t.y,
            c + w * tz + (x * ty - y * tx) + t.z]


def png_chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def write_png(path, width, height, rows, channels):
    """Minimal 8-bit RGB (3 channels) or grayscale (1 channel) PNG writer."""
    color_type = 2 if channels == 3 else 0
    pixels = b"".join(b"\0" + row for row in rows)
    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", header)
        + png_chunk(b"IDAT", zlib.compress(pixels, 6))
        + png_chunk(b"IEND", b"")
    )


def save_rgb(msg, path):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise RuntimeError(f"Unsupported RGB encoding: {msg.encoding}")
    raw = bytes(msg.data)
    rows = []
    for y in range(msg.height):
        row = np.frombuffer(raw, dtype=np.uint8, count=msg.width * 3,
                            offset=y * msg.step).reshape(msg.width, 3)
        if msg.encoding == "bgr8":
            row = row[:, ::-1]
        rows.append(row.tobytes())
    write_png(path, msg.width, msg.height, rows, channels=3)


def save_depth(msg, npy_path, preview_path):
    enc = msg.encoding.upper()
    if enc == "32FC1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        scale = 1.0
    elif enc == "16UC1":
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        scale = 0.001  # ROS 16UC1 depth is conventionally millimetres.
    else:
        raise RuntimeError(f"Unsupported depth encoding: {msg.encoding}")
    raw = bytes(msg.data)
    depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=raw,
                       strides=(msg.step, dtype.itemsize)).astype(np.float32) * scale
    np.save(npy_path, depth)  # Always save metres; keep NaN/inf if present.
    good = np.isfinite(depth) & (depth > 0)
    preview = np.zeros(depth.shape, dtype=np.uint8)
    if good.any():
        near, far = np.percentile(depth[good], [2, 98])
        if far <= near:
            far = near + 0.001
        preview[good] = (255 - np.clip((depth[good] - near) /
                                       (far - near), 0, 1) * 254).astype(np.uint8)
    write_png(preview_path, msg.width, msg.height,
              [row.tobytes() for row in preview], channels=1)
    return enc


class CaptureNode(Node):
    def __init__(self):
        super().__init__(
            "wrist_capture",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.lock = threading.Lock()
        self.rgb = deque(maxlen=40)
        self.depth = deque(maxlen=40)
        self.joints = deque(maxlen=100)
        self.info = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/wrist_cam/image_raw",
                                 lambda m: self.remember(self.rgb, m), qos)
        self.create_subscription(Image, "/wrist_cam/depth/image_raw",
                                 lambda m: self.remember(self.depth, m), qos)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.remember(self.joints, m), qos)
        self.create_subscription(CameraInfo, "/wrist_cam/camera_info",
                                 self.on_info, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def remember(self, queue, msg):
        with self.lock:
            queue.append((time.monotonic(), msg))

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def snapshot(self):
        with self.lock:
            return list(self.rgb), list(self.depth), list(self.joints), self.info

    def get_frame_set(self, after):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rgbs, depths, joints, info = self.snapshot()
            fresh = [(received, m) for received, m in rgbs if received > after]
            if fresh and depths and joints and info is not None:
                _, rgb = fresh[-1]
                depth = min(depths, key=lambda p: abs(stamp_ns(p[1]) - stamp_ns(rgb)))[1]
                joint = min(joints, key=lambda p: abs(stamp_ns(p[1]) - stamp_ns(rgb)))[1]
                depth_delta = abs(stamp_ns(depth) - stamp_ns(rgb)) / 1e9
                joint_delta = abs(stamp_ns(joint) - stamp_ns(rgb)) / 1e9
                if depth_delta <= 0.20 and joint_delta <= 0.20:
                    return rgb, depth, joint, info, depth_delta, joint_delta
            time.sleep(0.05)
        raise RuntimeError("Timed out matching fresh RGB, depth and joint data (check /clock and topics)")

    def wait_stopped(self):
        """Avoid capturing while a robot joint is still changing."""
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            now = time.monotonic()
            _, _, samples, _ = self.snapshot()
            recent = [(t, dict(zip(msg.name, msg.position)))
                      for t, msg in samples if t > now - 0.5]
            if len(recent) >= 3 and recent[0][0] < now - 0.35:
                if all(all(name in angles for name in JOINTS) for _, angles in recent):
                    max_change = max(max(angles[name] for _, angles in recent) -
                                     min(angles[name] for _, angles in recent)
                                     for name in JOINTS)
                    if max_change < 0.006:
                        return
            time.sleep(0.05)
        raise RuntimeError("Robot joints did not settle; verify /joint_states")

    def camera_transform(self, stamp, target="base_link", source="camera_optical_frame"):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                return self.tf_buffer.lookup_transform(
                    target, source, Time.from_msg(stamp))
            except TransformException:
                time.sleep(0.05)
        raise RuntimeError(f"No TF: {target} <- {source} at image timestamp")


def save_info(info, path):
    data = {
        "image_width": int(info.width), "image_height": int(info.height),
        "camera_name": "wrist_cam", "distortion_model": info.distortion_model,
        "distortion_coefficients": {"rows": 1, "cols": len(info.d),
                                    "data": [float(x) for x in info.d]},
        "camera_matrix": {"rows": 3, "cols": 3,
                          "data": [float(x) for x in info.k]},
        "rectification_matrix": {"rows": 3, "cols": 3,
                                 "data": [float(x) for x in info.r]},
        "projection_matrix": {"rows": 3, "cols": 4,
                              "data": [float(x) for x in info.p]},
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--cube-world", nargs=3, type=float, default=[0.50, 0.0, 0.04],
                        metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).resolve().parent.parent / "resuts",
        help="Dataset folder (default: ur5e_isaac_bridge/resuts)",
    )
    args = parser.parse_args()
    if not 1 <= args.count <= 5:
        parser.error("--count must be between 1 and 5")
    # Capture count is the total requested. A one-view test can be resumed later.
    records_path = args.output / "poses.jsonl"
    existing = []
    if records_path.exists():
        try:
            existing = [json.loads(line) for line in records_path.read_text().splitlines()
                        if line.strip()]
        except json.JSONDecodeError:
            parser.error(f"Invalid JSON in {records_path}")
    if existing and any(row.get("cube_center_world_xyz") != args.cube_world for row in existing):
        parser.error("Cube position differs from the dataset already in this directory")
    if len(existing) >= args.count:
        print(f"Already have {len(existing)} capture(s) in {args.output}")
        return

    rclpy.init(args=[])
    node = CaptureNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        for folder in ("rgb", "depth", "depth_vis"):
            (args.output / folder).mkdir(parents=True, exist_ok=True)
        print(f"Saving to: {args.output}")
        print("In RViz, stop the arm with the cube visible before pressing Enter.")
        for index in range(len(existing) + 1, args.count + 1):
            input(f"View {index}/{args.count}: press Enter to capture (Ctrl+C to quit) ... ")
            node.wait_stopped()
            pressed = time.monotonic()
            rgb, depth, joint, info, depth_dt, joint_dt = node.get_frame_set(pressed)
            pose = node.camera_transform(rgb.header.stamp)
            world_to_base = node.camera_transform(rgb.header.stamp,
                                                  target="base_link", source="world")
            cube_base = transform_point(world_to_base, args.cube_world)
            angles = dict(zip(joint.name, joint.position))
            if any(name not in angles for name in JOINTS):
                raise RuntimeError("Missing one or more UR5e joint names on /joint_states")
            save_rgb(rgb, args.output / "rgb" / f"img_{index:02d}.png")
            encoding = save_depth(depth, args.output / "depth" / f"depth_{index:02d}.npy",
                                  args.output / "depth_vis" / f"depth_{index:02d}.png")
            save_info(info, args.output / "camera_info.yaml")
            record = {
                "image": f"rgb/img_{index:02d}.png",
                "depth": f"depth/depth_{index:02d}.npy",
                "depth_preview": f"depth_vis/depth_{index:02d}.png",
                "image_stamp": {"sec": rgb.header.stamp.sec,
                                "nanosec": rgb.header.stamp.nanosec},
                "depth_encoding": encoding,
                "cube_center_world_xyz": args.cube_world,
                "cube_center_base_xyz": cube_base,
                "depth_stamp_delta_sec": round(depth_dt, 5),
                "joint_stamp_delta_sec": round(joint_dt, 5),
                "joint_names": JOINTS,
                "joint_positions": [float(angles[name]) for name in JOINTS],
                "camera_pose": {
                    "parent_frame": "base_link", "child_frame": "camera_optical_frame",
                    "translation_xyz": vec3(pose.transform.translation),
                    "quaternion_xyzw": quat(pose.transform.rotation),
                },
            }
            with records_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record) + "\n")
            print(f"Saved view {index}: RGB, depth, joints and camera pose")
        print(f"Finished: {args.count} view(s) in {args.output}")
    except KeyboardInterrupt:
        print("Capture cancelled.")
    except (RuntimeError, EOFError) as exc:
        raise SystemExit(f"Capture failed: {exc}") from exc
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=1)


if __name__ == "__main__":
    main()
