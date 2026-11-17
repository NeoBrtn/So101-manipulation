#!/usr/bin/env python3
"""perception_node.py - phase 2: the standing bottle, found by its cap, in the robot frame.

For each camera frame (latest only, at publish_hz):
  1. HSV segmentation of the cap colour (green); the bottle body is clear plastic and
     the real arm is blue, so the cap is the reliable part
  2. Centroid (u, v) of the largest green blob (rejected if cut by the image edge)
  3. Pixel -> world ray (camera pose from TF), intersected with the plane of the cap's
     top: seen from above, the top of a standing bottle's cap is what the camera sees
  4. Tracker: rejects partly hidden blobs and one-off jumps (so101_vision.tracking)

Publishes (frame "world" = base_link):
    /bottle_position      geometry_msgs/PoseStamped, the bottle's axis on the table
    /perception/markers   visualization_msgs/MarkerArray (RViz)
"""
import cv2
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from so101_vision.camera_model import CameraModel, image_to_rgb
from so101_vision.geometry import pixel_to_plane
from so101_vision.segmentation import hsv_mask, largest_blob
from so101_vision.tracking import Tracker

WORLD_FRAME = "world"


class BottlePerception(rclpy.node.Node):
    def __init__(self):
        super().__init__("bottle_perception")
        p = self.declare_parameter
        camera = p("camera", "overhead_cam").value
        self._table_z = p("table_z", -0.18).value
        self._cap_top = p("cap_top", 0.222).value              # m above the bottle's bottom
        self._bottle_height = self._cap_top
        flat = p("cap_hsv", [60, 110, 35, 88, 255, 255]).value  # green (sim and real cap)
        self._hsv = [(tuple(flat[i:i + 3]), tuple(flat[i + 3:i + 6])) for i in range(0, len(flat), 6)]
        self._min_area = p("min_blob_area", 15).value
        self._tracker = Tracker(min_area_ratio=0.5)
        self._camera = CameraModel(self, camera, WORLD_FRAME)
        self._latest = None
        self._last_stamp = None
        self.create_subscription(Image, f"/{camera}/image_raw", self._cb_image, 1)
        self._pub = self.create_publisher(PoseStamped, "/bottle_position", 10)
        self._pub_markers = self.create_publisher(MarkerArray, "/perception/markers", 10)
        self.create_timer(1.0 / p("publish_hz", 10.0).value, self._process)
        self.get_logger().info(f"Bottle perception ready (camera: /{camera}).")

    def _cb_image(self, msg: Image):
        self._latest = msg

    def _process(self):
        msg = self._latest
        if msg is None or msg.header.stamp == self._last_stamp or self._camera.K is None:
            return
        self._last_stamp = msg.header.stamp
        T_wc = self._camera.update_pose(msg.header.frame_id)
        if T_wc is None:
            return
        hsv = cv2.cvtColor(image_to_rgb(msg), cv2.COLOR_RGB2HSV)
        blob = largest_blob(hsv_mask(hsv, self._hsv), self._min_area)
        if blob is None or blob.touches_border(hsv.shape):
            return
        top = pixel_to_plane(self._camera.K, T_wc, blob.center, self._table_z + self._cap_top)
        if top is None or not self._tracker.update(top, blob.area):
            return
        x, y, _ = self._tracker.estimate
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = msg.header.stamp, WORLD_FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(self._table_z)
        pose.pose.orientation.w = 1.0
        self._pub.publish(pose)
        self._publish_marker(pose)

    def _publish_marker(self, pose):
        m = Marker()
        m.header = pose.header
        m.ns, m.id, m.type, m.action = "bottle", 0, Marker.CYLINDER, Marker.ADD
        m.pose = pose.pose
        m.pose.position.z = self._table_z + self._bottle_height / 2
        m.scale.x = m.scale.y = 0.06
        m.scale.z = self._bottle_height
        m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.8, 0.4, 0.5
        m.lifetime.sec = 1
        self._pub_markers.publish(MarkerArray(markers=[m]))


def main(args=None):
    rclpy.init(args=args)
    node = BottlePerception()
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
