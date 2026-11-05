#!/usr/bin/env python3
"""camera_stream_node.py - a network camera as a ROS camera, for the real setup.

Any stream OpenCV can open: DroidCam on a phone (http://<phone-ip>:4747/video), ffmpeg
serving a laptop webcam, an RTSP camera... Publishes the same topics as the simulated
camera, so the rest of the stack (perception) does not know the difference:

    /<camera_name>/image_raw     rgb8, undistorted
    /<camera_name>/camera_info   K of the undistorted image, no distortion
    TF world -> <camera_name>_optical_frame

Camera pose, so the camera can be put anywhere:
  - marker_xy set: LIVE, from an ArUco marker lying on the table at a known spot (see
    aruco_pose.py for the convention). A single marker gives two candidate poses per
    frame (mirror ambiguity): the one consistent with the current estimate is kept; the
    start uses the median of the best fits. Filtered over POSE_WINDOW frames; if the
    camera is moved, POSE_WINDOW // 2 frames agreeing on a new pose re-anchor it. The
    last pose is kept while the marker is hidden (e.g. by the arm).
  - else: the fixed world_T_camera of the calibration file (calibrate_camera pose), if any.

Lens (calibration file `camera_matrix`, `distortion`, from calibrate_camera intrinsics):
once per phone and resolution, it does not change when the camera moves. Without it, a
typical phone lens is assumed: it works, but the 3D estimates are less accurate.
"""
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rclpy.node
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from so101_driver.aruco_pose import camera_T_marker_candidates, guess_K, make_detector, world_T_marker

WORLD_FRAME = "world"
POSE_WINDOW = 30          # marker poses filtered together (median position, mean rotation)
POSE_JUMP = 0.03          # m: further than this from the current estimate = outlier...
MOVED_FRAMES = 15         # ...unless this many frames in a row agree on it: the camera moved


def load_calibration(path):
    if not path or not Path(path).is_file():
        return None
    with open(path) as f:
        return yaml.safe_load(f)


class CameraStreamNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("camera_stream")
        self._url: str = self.declare_parameter("url", "").value
        self._camera: str = self.declare_parameter("camera_name", "overhead_cam").value
        calib_path: str = self.declare_parameter("calibration", "").value
        fps: float = self.declare_parameter("fps", 15.0).value
        # Published image scale: a 1280x720 RGB frame is 2.7 MB, too slow to pass around in
        # Python at 10 Hz. The marker is still found on the full frame (precision).
        self._scale: float = self.declare_parameter("publish_scale", 0.5).value
        marker_xy = list(self.declare_parameter("marker_xy", [0.0, 0.0]).value)
        self._marker_z: float = self.declare_parameter("marker_z", 0.0).value
        self._marker_size: float = self.declare_parameter("marker_size", 0.12).value
        self._marker_id: int = self.declare_parameter("marker_id", 0).value
        if not self._url:
            raise RuntimeError("camera_stream: set the `url` parameter (e.g. http://<phone-ip>:4747/video)")

        self._frame_id = f"{self._camera}_optical_frame"
        self._calib = load_calibration(calib_path)
        self._maps = None   # undistortion maps, built on the first frame
        self._K = None
        self._live = any(v != 0.0 for v in marker_xy)
        self._world_T_marker = world_T_marker(marker_xy, self._marker_z)
        self._poses = deque(maxlen=POSE_WINDOW)
        self._outliers = []
        self._world_T_cam = None
        self._detector = make_detector()

        if self._calib is None or "camera_matrix" not in self._calib:
            self.get_logger().warn("Lens not calibrated: assuming a typical phone lens (less accurate). "
                                   "Once per phone: ros2 run so101_driver calibrate_camera intrinsics --help")
        if self._live:
            self.get_logger().info(f"Camera pose: live, from ArUco id {self._marker_id} "
                                   f"({self._marker_size * 1000:.0f} mm) at {marker_xy}")
        elif self._calib is not None and "world_T_camera" in self._calib:
            self._static_tf = StaticTransformBroadcaster(self)
            self._static_tf.sendTransform(self._tf(np.array(self._calib["world_T_camera"], float).reshape(4, 4)))
            self.get_logger().info("Camera pose: fixed, from the calibration file")
        else:
            self.get_logger().warn("No camera pose: set MARKER_XY (live ArUco) in docker/.env")

        self._tf_pub = TransformBroadcaster(self)
        self._pub_image = self.create_publisher(Image, f"/{self._camera}/image_raw", 10)
        self._pub_info = self.create_publisher(CameraInfo, f"/{self._camera}/camera_info", 10)

        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.create_timer(1.0 / fps, self._publish)
        self.get_logger().info(f"Camera stream {self._url} -> /{self._camera}/image_raw")

    def _read_loop(self):
        """Keeps only the newest frame (no lag build-up), reconnects if the stream drops."""
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self._url)
            if not cap.isOpened():
                self.get_logger().warn(f"Cannot open {self._url}, retrying...", throttle_duration_sec=10.0)
                self._stop.wait(2.0)
                continue
            self.get_logger().info("Stream connected.")
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn("Stream lost, reconnecting...")
                    break
                with self._lock:
                    self._latest = frame
            cap.release()

    def _setup_undistort(self, w, h):
        c = self._calib
        if c is None or "camera_matrix" not in c:
            self._K = guess_K(w, h)
            return
        if (c["image_width"], c["image_height"]) != (w, h):
            raise RuntimeError(f"Lens calibrated for {c['image_width']}x{c['image_height']}, stream is {w}x{h}")
        K = np.array(c["camera_matrix"], dtype=float).reshape(3, 3)
        D = np.array(c["distortion"], dtype=float)
        # alpha=0: crop to valid pixels only. Same optical frame, so poses still hold.
        self._K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0)
        self._maps = cv2.initUndistortRectifyMap(K, D, None, self._K, (w, h), cv2.CV_16SC2)

    @staticmethod
    def _filtered(poses):
        Ts = np.array(poses)
        T = np.eye(4)
        T[:3, :3] = Rotation.from_matrix(Ts[:, :3, :3]).mean().as_matrix()
        T[:3, 3] = np.median(Ts[:, :3, 3], axis=0)
        return T

    def _update_pose(self, gray):
        """Camera pose from the marker in this (undistorted) frame, filtered."""
        cands = camera_T_marker_candidates(self._detector, gray, self._K, np.zeros(5),
                                           self._marker_id, self._marker_size)
        if not cands:
            return  # marker hidden: keep the last pose
        poses = [self._world_T_marker @ np.linalg.inv(T) for T, _ in cands]

        if self._world_T_cam is None:
            # Start: best fit of each frame, median once there are enough (rejects the
            # frames where the mirrored pose happened to fit better)
            self._poses.append(poses[0])
            if len(self._poses) >= MOVED_FRAMES:
                ref = np.median(np.array(self._poses)[:, :3, 3], axis=0)
                keep = [T for T in self._poses if np.linalg.norm(T[:3, 3] - ref) < POSE_JUMP]
                self._poses = deque(keep, maxlen=POSE_WINDOW)
                self._world_T_cam = self._filtered(keep)
                p = self._world_T_cam[:3, 3]
                self.get_logger().info(f"Marker found: camera at x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} m "
                                       f"({len(keep)}/{MOVED_FRAMES} frames agree)")
            return

        # Running: the candidate consistent with the current estimate
        cur = self._world_T_cam[:3, 3]
        T = min(poses, key=lambda P: np.linalg.norm(P[:3, 3] - cur))
        if np.linalg.norm(T[:3, 3] - cur) < POSE_JUMP:
            self._outliers.clear()
            self._poses.append(T)
            self._world_T_cam = self._filtered(self._poses)
            return
        # Far from the current estimate: noise, unless it persists (the camera was moved)
        self._outliers.append(poses[0])
        if len(self._outliers) >= MOVED_FRAMES:
            pts = np.array(self._outliers)[:, :3, 3]
            if np.ptp(pts, axis=0).max() < 2 * POSE_JUMP:
                self._poses = deque(self._outliers, maxlen=POSE_WINDOW)
                self._world_T_cam = self._filtered(self._poses)
                p = self._world_T_cam[:3, 3]
                self.get_logger().info(f"Camera moved: now at x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} m")
            self._outliers.clear()

    def _publish(self):
        with self._lock:
            frame, self._latest = self._latest, None
        if frame is None:
            return
        h, w = frame.shape[:2]
        if self._K is None:
            self._setup_undistort(w, h)
        if self._maps is not None:
            frame = cv2.remap(frame, *self._maps, cv2.INTER_LINEAR)
        if self._live:
            self._update_pose(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        K = self._K
        if self._scale != 1.0:
            frame = cv2.resize(frame, None, fx=self._scale, fy=self._scale, interpolation=cv2.INTER_AREA)
            h, w = frame.shape[:2]
            K = self._K.copy()
            K[:2] *= self._scale   # fx, fy, cx, cy scale with the image
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        stamp = self.get_clock().now().to_msg()
        if self._live and self._world_T_cam is not None:
            tf = self._tf(self._world_T_cam)
            tf.header.stamp = stamp
            self._tf_pub.sendTransform(tf)

        img = Image()
        img.header.stamp, img.header.frame_id = stamp, self._frame_id
        img.height, img.width, img.encoding, img.step = h, w, "rgb8", w * 3
        img.data = rgb.tobytes()
        self._pub_image.publish(img)

        info = CameraInfo()
        info.header = img.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = K.flatten().tolist()
        info.r = np.eye(3).flatten().tolist()
        info.p = np.hstack([K, np.zeros((3, 1))]).flatten().tolist()
        self._pub_info.publish(info)

    def _tf(self, T):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = WORLD_FRAME, self._frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = map(float, q)
        return tf

    def destroy_node(self):
        self._stop.set()
        self._reader.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
