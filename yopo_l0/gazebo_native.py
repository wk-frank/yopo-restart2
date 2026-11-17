"""Opt-in Gazebo rendered cameras; no scene-card depth fallback or PX4 writes."""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .sensor import CameraConfig, DepthFrame


def depth_topic(instance: int, world: str = "default") -> str:
    return f"/world/{world}/model/x500_{int(instance)}/link/base_link/sensor/yopo_depth/depth_image"


def prepare_native_scene(world_path, base_model_path, camera: CameraConfig, *, record: bool = True) -> dict:
    """Modify only the run's world and ephemeral container model before spawning PX4."""
    world_path, base_model_path = Path(world_path), Path(base_model_path)
    root = ET.fromstring(world_path.read_text(encoding="utf-8-sig"))
    world = root.find("world")
    if world is None:
        raise ValueError("SDF world missing")
    cylinders=[m for m in world.findall('model') if m.find('link/collision/geometry/cylinder') is not None]
    if len(cylinders)>5000:
        group=ET.SubElement(world,'model',name='observed_obstacle_field')
        ET.SubElement(group,'static').text='true'
        link=ET.SubElement(group,'link',name='field')
        for index,model in enumerate(cylinders):
            for tag in ('collision','visual'):
                child=model.find('link/'+tag)
                child.set('name',f'{tag}_{index}')
                ET.SubElement(child,'pose').text=model.findtext('pose','0 0 0 0 0 0')
                link.append(child)
            world.remove(model)
    if len(cylinders)>50000:
        from .forest_mesh import batch_trunk_visuals
        batch_trunk_visuals(world,world_path.parent/'trunk_meshes')
    # Explicit systems keep physics and PX4 sensors when a world plugin is added.
    for suffix, name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                         ("scene-broadcaster", "SceneBroadcaster"), ("imu", "Imu"),
                         ("magnetometer", "Magnetometer"), ("air-pressure", "AirPressure"),
                         ("navsat", "NavSat"), ("sensors", "Sensors")):
        filename = f"gz-sim-{suffix}-system"
        if not any(p.get("filename") == filename for p in world.findall("plugin")):
            plugin = ET.SubElement(world, "plugin", filename=filename, name=f"gz::sim::systems::{name}")
            if name == "Sensors":
                ET.SubElement(plugin, "render_engine").text = "ogre2"
    shadows = world.find("scene/shadows")
    if shadows is not None:
        shadows.text = "false"
    if record and world.find("model[@name='yopo_observer']") is None:
        model = ET.SubElement(world, "model", name="yopo_observer")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = "-24 -15 19 0 0.48 0.56"
        link = ET.SubElement(model, "link", name="camera_link")
        sensor = ET.SubElement(link, "sensor", name="flight_camera", type="camera")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "10"
        ET.SubElement(sensor, "topic").text = "/yopo/gazebo/flight_camera"
        cam = ET.SubElement(sensor, "camera")
        ET.SubElement(cam, "horizontal_fov").text = "1.57"
        image = ET.SubElement(cam, "image")
        for key, value in (("width", "1280"), ("height", "720"), ("format", "R8G8B8")):
            ET.SubElement(image, key).text = value
        clip = ET.SubElement(cam, "clip")
        ET.SubElement(clip, "near").text = "0.1"
        ET.SubElement(clip, "far").text = "200"
    world_path.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
    base = ET.fromstring(base_model_path.read_text(encoding="utf-8-sig"))
    link = base.find("model/link[@name='base_link']")
    if link is None:
        raise ValueError("x500 base_link missing")
    old = link.find("sensor[@name='yopo_depth']")
    if old is not None:
        link.remove(old)
    sensor = ET.SubElement(link, "sensor", name="yopo_depth", type="depth_camera")
    # +X forward, +Y left, +Z up, rigidly attached to the actual Gazebo body.
    ET.SubElement(sensor, "pose").text = " ".join(map(str, (*camera.mount_xyz_body, 0, 0, 0)))
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "15"
    cam = ET.SubElement(sensor, "camera")
    ET.SubElement(cam, "horizontal_fov").text = str(math.radians(camera.hfov_deg))
    image = ET.SubElement(cam, "image")
    for key, value in (("width", camera.width), ("height", camera.height), ("format", "R_FLOAT32")):
        ET.SubElement(image, key).text = str(value)
    lens = ET.SubElement(cam, "lens")
    intrinsics = ET.SubElement(lens, "intrinsics")
    for key, value in zip(("fx", "fy", "cx", "cy"), camera.intrinsics):
        ET.SubElement(intrinsics, key).text = str(value)
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = str(camera.min_range_m)
    ET.SubElement(clip, "far").text = str(camera.max_range_m)
    base_model_path.write_text(ET.tostring(base, encoding="unicode"), encoding="utf-8")
    meshes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (world_path.parent/'trunk_meshes').glob('*')}
    return {"depth_hz": 15, "camera": camera.__dict__, "renderer": "Gazebo Ogre2 EGL", "mesh_sha256":meshes,
            "record": record, "world_sha256": hashlib.sha256(world_path.read_bytes()).hexdigest(),
            "base_model_sha256": hashlib.sha256(base_model_path.read_bytes()).hexdigest()}


def image_stamp(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nsec) * 1e-9


def decode_gz_depth(msg, camera: CameraConfig) -> DepthFrame:
    """Decode Gazebo R_FLOAT32 (metres), retaining its simulation capture stamp."""
    from_size = (int(msg.height), int(msg.width))
    if from_size != (camera.height, camera.width):
        raise ValueError(f"Gazebo depth dimensions {from_size} mismatch")
    # gz-msgs10 PixelFormatType.R_FLOAT32, verified against installed protobuf.
    if int(msg.pixel_format_type) != 13:
        raise ValueError(f"Gazebo depth pixel format {msg.pixel_format_type}, expected R_FLOAT32=13")
    row_bytes = camera.width * 4
    step = int(msg.step) or row_bytes
    if step < row_bytes or len(msg.data) != step * camera.height:
        raise ValueError("Malformed Gazebo depth stride/data")
    depth = np.ndarray(from_size, dtype="<f4", buffer=msg.data, strides=(step, 4)).copy()
    depth[(depth <= 0) | ~np.isfinite(depth)] = np.nan
    return DepthFrame(image_stamp(msg), depth, camera.intrinsics)


class NativeDepthSource:
    """Gz transport subscriptions plus per-vehicle consumed-frame evidence."""

    def __init__(self, node, identities, camera: CameraConfig, out_dir, world="default"):
        from gz.msgs10.image_pb2 import Image
        self.camera, self.out_dir = camera, Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        (self.out_dir / "depth-samples").mkdir(exist_ok=True)
        self.archive_log = (self.out_dir / "depth-consumed.jsonl").open("w")
        self.last_saved = {}
        self.lock = threading.Lock()
        self.frames, self.stats, self.errors = {}, {}, []
        self.topics = {i["namespace"]: depth_topic(i["instance"], world) for i in identities}
        for ns, topic in self.topics.items():
            self.stats[ns] = {"received": 0, "consumed": 0, "unique_consumed": 0,
                              "max_age_s": 0.0, "first_stamp_s": None, "last_stamp_s": None,
                              "last_consumed_stamp_s": None, "finite_pixels": 0}
            def receive(msg, ns=ns):
                try:
                    frame = decode_gz_depth(msg, camera)
                    with self.lock:
                        self.frames[ns] = frame
                        stat = self.stats[ns]
                        stat["received"] += 1
                        stat["last_stamp_s"] = frame.stamp
                        if stat["first_stamp_s"] is None:
                            stat["first_stamp_s"] = frame.stamp
                except ValueError as exc:
                    with self.lock:
                        self.errors.append(str(exc))
            if not node.subscribe(Image, topic, receive):
                raise RuntimeError(f"Cannot subscribe {topic}")

    def frame(self, ns, now_s, *, origin_s=0.0, consume=False, max_age_s=0.20):
        with self.lock:
            frame = self.frames.get(ns)
            if self.errors:
                raise ValueError(self.errors[-1])
            if frame is None or now_s - frame.stamp > max_age_s or frame.stamp > now_s + 0.025:
                return None
            if consume:
                stat = self.stats[ns]
                stat["consumed"] += 1
                stat["max_age_s"] = max(stat["max_age_s"], now_s - frame.stamp)
                if stat["last_consumed_stamp_s"] != frame.stamp:
                    stat["unique_consumed"] += 1
                    stat["last_consumed_stamp_s"] = frame.stamp
                    digest = hashlib.sha256(frame.depth.tobytes()).hexdigest()
                    self.archive_log.write(json.dumps({"namespace": ns, "gazebo_stamp_s": frame.stamp,
                                                       "sha256": digest, "used_at_s": now_s}) + "\n")
                    if frame.stamp - self.last_saved.get(ns, -math.inf) >= 5.0:
                        np.savez_compressed(self.out_dir / "depth-samples" / f"{ns.strip('/')}-{stat['unique_consumed']:05d}.npz",
                                            depth=frame.depth, K=frame.K, gazebo_stamp_s=frame.stamp)
                        self.last_saved[ns] = frame.stamp
                        self.archive_log.flush()
                stat["finite_pixels"] += int(np.isfinite(frame.depth).sum())
            return DepthFrame(frame.stamp - origin_s, frame.depth, frame.K)

    def summary(self):
        with self.lock:
            return {"source": "gazebo-native-depth", "fallback_used": False,
                    "topics": dict(self.topics), "vehicles": json.loads(json.dumps(self.stats)),
                    "decode_errors": list(self.errors)}

    def close(self):
        self.archive_log.close()


class GazeboFlightRecorder:
    """Encode images published by a real Gazebo camera in this running world."""

    def __init__(self, node, out_dir, world="default"):
        import subprocess
        from gz.msgs10.image_pb2 import Image

        self.node, self.world = node, world
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.path = self.out / "gazebo-flight.mp4"
        self.log = (self.out / "gazebo-video-frames.jsonl").open("w")
        self.stderr = (self.out / "gazebo-video-ffmpeg.log").open("w")
        self.process = subprocess.Popen(
            ["ffmpeg", "-nostdin", "-loglevel", "warning", "-y", "-f", "rawvideo",
             "-pixel_format", "rgb24", "-video_size", "1280x720", "-framerate", "10",
             "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "19",
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.path)],
            stdin=subprocess.PIPE, stderr=self.stderr)
        self.lock = threading.Lock()
        self.frames, self.first, self.last = 0, None, None
        self.errors = []
        self.active, self.closed = False, False
        self._last_follow = -math.inf
        self._hash = hashlib.sha256()

        def receive(msg):
            with self.lock:
                if not self.active or self.closed:
                    return
                stamp = image_stamp(msg)
                if self.last is not None and stamp <= self.last:
                    return
                if (msg.width, msg.height, msg.pixel_format_type) != (1280, 720, 3):
                    self.errors.append("Unexpected Gazebo RGB image format")
                    return
                raw = bytes(msg.data)
                if len(raw) != 1280 * 720 * 3:
                    self.errors.append("Unexpected Gazebo RGB data size")
                    return
                try:
                    self.process.stdin.write(raw)
                    self._hash.update(raw)
                    self.log.write(json.dumps({"frame": self.frames, "gazebo_stamp_s": stamp,
                                               "sha256": hashlib.sha256(raw).hexdigest()}) + "\n")
                    if self.first is None:
                        self.first = stamp
                    if self.frames % 100 == 0:
                        import cv2
                        cv2.imwrite(str(self.out / f"gazebo-frame-{self.frames:05d}.jpg"),
                                    np.frombuffer(raw, np.uint8).reshape(720, 1280, 3)[..., ::-1])
                        self.log.flush()
                    self.frames += 1
                    self.last = stamp
                except OSError as exc:
                    self.errors.append(str(exc))

        if not node.subscribe(Image, "/yopo/gazebo/flight_camera", receive):
            raise RuntimeError("Cannot subscribe Gazebo observer camera")

    def follow(self, positions, now_s):
        if now_s - self._last_follow < 0.09 or not positions:
            return
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        mean = np.mean(list(positions.values()), axis=0)
        request = Pose(name="yopo_observer")
        request.position.x, request.position.y, request.position.z = float(mean[0] - 24), float(mean[1] - 15), 19.0
        pitch, yaw = 0.48, 0.56
        request.orientation.x = -math.sin(pitch/2) * math.sin(yaw/2)
        request.orientation.y = math.sin(pitch/2) * math.cos(yaw/2)
        request.orientation.z = math.cos(pitch/2) * math.sin(yaw/2)
        request.orientation.w = math.cos(pitch/2) * math.cos(yaw/2)
        self.node.request(f"/world/{self.world}/set_pose", request, Pose, Boolean, 50)
        self._last_follow = now_s

    def close(self):
        with self.lock:
            self.closed = True
            self.process.stdin.close()
            self.log.close()
        rc = self.process.wait(timeout=60)
        self.stderr.close()
        result = {"source": "gazebo-native-rgb-camera", "topic": "/yopo/gazebo/flight_camera",
                  "file": self.path.name, "frames": self.frames, "fps": 10,
                  "first_gazebo_stamp_s": self.first, "last_gazebo_stamp_s": self.last,
                  "raw_frame_stream_sha256": self._hash.hexdigest(),
                  "ffmpeg_exit_code": rc, "errors": self.errors,
                  "mp4_sha256": hashlib.sha256(self.path.read_bytes()).hexdigest() if self.path.is_file() else None}
        (self.out / "gazebo-video.json").write_text(json.dumps(result, indent=2) + "\n")
        return result

