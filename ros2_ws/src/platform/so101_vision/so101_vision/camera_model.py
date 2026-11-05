"""ROS camera model: intrinsics from camera_info, pose from TF, image as numpy.

The camera pose is looked up on every frame, so it may move: in simulation the driver
publishes the ground truth as a static TF; on the real setup camera_stream_node
estimates it live from an ArUco marker on the table.
"""
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from so101_vision.geometry import quaternion_to_matrix


class CameraModel:
    def __init__(self, node, camera: str, world_frame: str = "world"):
        self._node = node
        self.world_frame = world_frame
        self.K = None
        self.T_wc = None
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node)
        node.create_subscription(CameraInfo, f"/{camera}/camera_info", self._cb_info, 1)

    @property
    def ready(self) -> bool:
        return self.K is not None and self.T_wc is not None

    def _cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def update_pose(self, frame_id: str):
        """Latest camera -> world pose (keeps the last one if TF has nothing)."""
        try:
            tf = self._tf.lookup_transform(self.world_frame, frame_id, Time())
        except TransformException as e:
            if self.T_wc is None:
                self._node.get_logger().warn(f"No TF {self.world_frame} -> {frame_id} yet: {e}",
                                             throttle_duration_sec=5.0)
            return self.T_wc
        t, r = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quaternion_to_matrix(r.x, r.y, r.z, r.w)
        T[:3, 3] = [t.x, t.y, t.z]
        self.T_wc = T
        return T


def image_to_rgb(msg: Image) -> np.ndarray:
    """sensor_msgs/Image (rgb8 or bgr8) -> HxWx3 RGB array, without cv_bridge."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported encoding {msg.encoding}")
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1] if msg.encoding == "bgr8" else img


def rgb_to_image(rgb: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding, msg.step = "rgb8", rgb.shape[1] * 3
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg
