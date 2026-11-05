#!/usr/bin/env python3
"""perception_node.py - phase 1: red cube and drop box, in the robot frame.

For each camera frame (latest only, at publish_hz):
  1. HSV segmentation: red cube (hue wraps around 0/180: two ranges), brown box
  2. Centroid (u, v) of the largest blob of each colour
  3. Pixel -> world ray: d = R_wc K^-1 [u, v, 1], camera pose from TF
  4. Depth from a plane assumption: ray intersected with z = cube centre height
     (the cube lies on the table) or z = box mid-height
  5. Tracker: rejects partly hidden blobs and one-off jumps (see so101_vision.tracking)

Why HSV: two saturated colours (red cube, brown box) on a grey floor with a grey arm.
Thresholds are cheap, deterministic, and easy to re-tune on the real camera (the sim
red already matches the printed PLA as seen by the phone camera).

Publishes (frame "world" = base_link):
    /object_position      geometry_msgs/PoseStamped, cube centre
    /drop_box_position    geometry_msgs/PoseStamped, box centre
    /perception/markers   visualization_msgs/MarkerArray (RViz)
    /perception/debug_image
"""
import cv2
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from so101_vision.camera_model import CameraModel, image_to_rgb, rgb_to_image
from so101_vision.geometry import pixel_to_plane
from so101_vision.segmentation import hsv_mask, largest_blob
from so101_vision.tracking import Tracker

WORLD_FRAME = "world"


def hsv_ranges(flat):
    """[h, s, v, h, s, v, ...] (groups of 6: low, high) -> [((h, s, v), (h, s, v)), ...]"""
    if len(flat) % 6:
        raise ValueError(f"HSV ranges: expected groups of 6 values, got {len(flat)}")
    return [(tuple(flat[i:i + 3]), tuple(flat[i + 3:i + 6])) for i in range(0, len(flat), 6)]


class Target:
    """One object to find: colour, plane height, tracker, output topic, marker."""

    def __init__(self, node, name, topic, hsv, plane_z, min_area_ratio, size, color):
        self.name, self.hsv, self.plane_z, self.size, self.color = name, hsv, plane_z, size, color
        self.tracker = Tracker(min_area_ratio=min_area_ratio)
        self.pub = node.create_publisher(PoseStamped, topic, 10)
        self.blob = None   # accepted detection of this frame (for the debug image)


class PerceptionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("perception")
        p = self.declare_parameter
        camera = p("camera", "external_cam").value
        self._min_area = p("min_blob_area", 15).value
        self._publish_debug = p("publish_debug_image", True).value
        cube_size = p("cube_size", 0.05).value
        box_size = p("box_size", [0.10, 0.10, 0.05]).value
        self._targets = [
            # Partly hidden blobs are rejected; stricter for the box: it is big and
            # static, so any hidden part shifts its centroid.
            Target(self, "cube", "/object_position",
                   hsv_ranges(p("cube_hsv", [0, 120, 50, 6, 255, 255, 160, 120, 50, 180, 255, 255]).value),
                   cube_size / 2, 0.6, [cube_size] * 3, (0.80, 0.05, 0.22)),
            Target(self, "drop_box", "/drop_box_position",
                   hsv_ranges(p("box_hsv", [7, 80, 25, 25, 255, 200]).value),
                   box_size[2] / 2, 0.9, box_size, (0.35, 0.20, 0.12)),
        ]

        self._camera = CameraModel(self, camera, WORLD_FRAME)
        self._latest = None
        self._last_stamp = None
        self.create_subscription(Image, f"/{camera}/image_raw", self._cb_image, 1)
        self._pub_markers = self.create_publisher(MarkerArray, "/perception/markers", 10)
        self._pub_debug = self.create_publisher(Image, "/perception/debug_image", 1)
        self.create_timer(1.0 / p("publish_hz", 10.0).value, self._process)
        self.get_logger().info(f"Perception ready (camera: /{camera}).")

    def _cb_image(self, msg: Image):
        self._latest = msg   # processed at publish_hz, never a backlog

    def _process(self):
        msg = self._latest
        if msg is None or msg.header.stamp == self._last_stamp or self._camera.K is None:
            return
        self._last_stamp = msg.header.stamp
        T_wc = self._camera.update_pose(msg.header.frame_id)
        if T_wc is None:
            return

        rgb = image_to_rgb(msg)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        markers = MarkerArray()
        for i, target in enumerate(self._targets):
            target.blob = largest_blob(hsv_mask(hsv, target.hsv), self._min_area)
            if target.blob is None:
                continue   # not seen: consumers keep the last known value
            if target.blob.touches_border(hsv.shape):
                self.get_logger().warn(f"{target.name} at the image edge (partly out of view): ignored",
                                       throttle_duration_sec=5.0)
                target.blob = None
                continue
            p = pixel_to_plane(self._camera.K, T_wc, target.blob.center, target.plane_z)
            if p is None or not target.tracker.update(p, target.blob.area):
                target.blob = None
                continue
            pose = self._pose(target.tracker.estimate, msg.header.stamp)
            target.pub.publish(pose)
            markers.markers.append(self._marker(i, target, pose))
        if markers.markers:
            self._pub_markers.publish(markers)
        if self._publish_debug:
            self._publish_debug_image(rgb, msg.header)

    @staticmethod
    def _pose(position, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = WORLD_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
        msg.pose.orientation.w = 1.0
        return msg

    @staticmethod
    def _marker(i, target, pose):
        m = Marker()
        m.header = pose.header
        m.ns, m.id, m.type, m.action = target.name, i, Marker.CUBE, Marker.ADD
        m.pose = pose.pose
        m.scale.x, m.scale.y, m.scale.z = map(float, target.size)
        m.color.r, m.color.g, m.color.b = target.color
        m.color.a = 0.8
        m.lifetime.sec = 1
        return m

    def _publish_debug_image(self, rgb, header):
        debug = rgb.copy()
        for target in self._targets:
            if target.blob is not None:
                u, v = (int(round(c)) for c in target.blob.center)
                cv2.drawMarker(debug, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)
                cv2.putText(debug, target.name, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
        self._pub_debug.publish(rgb_to_image(debug, header))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
