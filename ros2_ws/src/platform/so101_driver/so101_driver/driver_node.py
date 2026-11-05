#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.

    /joint_command  (sub)  sensor_msgs/JointState: radians, gripper 0-100 %
    /joint_states   (pub)  radians (gripper too, URDF joint), 50 Hz, right after each step
    /<camera>/image_raw, /<camera>/camera_info, TF world -> <camera>_optical_frame
                           simulation only (on the real arm: camera_stream_node)
    /reset_object   (srv)  simulation only: new random object pose

Real arm: the calibrated zero of each joint (middle of its recorded range) is not
exactly the URDF zero; the offsets measured by `measure_zero` are applied here, so
the IK and the simulation share the same angles.
"""
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import rclpy.node
import serial
import yaml
from geometry_msgs.msg import TransformStamped
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from so101_sim import ASSETS_DIR, SO101Sim

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Gripper joint limits in the URDF (rad).
# LeRobot convention: 0 % = closed (lower), 100 % = open (upper).
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
# Real arm: per-joint offsets (rad) between the calibrated zero (middle of each joint's
# recorded range) and the URDF zero pose. Written by `ros2 run so101_driver measure_zero`.
ZERO_OFFSETS_FILE = Path("/ros2_ws/calibration/arm_zero_offsets.yaml")
DEFAULT_PORT = "/dev/ttyACM0"

MAX_BUS_FAILURES = 25  # consecutive failed control steps (0.5 s) before giving up
CONTROL_HZ = 50.0  # /joint_states is published on every control step, right after it
WORLD_FRAME = "world"


def _gripper_pct_to_rad(pct: float) -> float:
    lo, hi = GRIPPER_RANGE_RAD
    return lo + min(max(pct, 0.0), 100.0) / 100.0 * (hi - lo)


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        # Passive: torque off, no commands sent, only /joint_states (move the arm by hand).
        self._passive: bool = self.declare_parameter("passive", False).value
        # Simulation only: scene file in so101_sim's assets ("" = default scene_tek5.xml),
        # camera to publish, and where to place the object ([0, 0] = scene default).
        self._scene: str = self.declare_parameter("scene", "").value
        self._camera: str = self.declare_parameter("camera_name", "external_cam").value
        self._camera_size = (
            self.declare_parameter("camera_width", 320).value,
            self.declare_parameter("camera_height", 240).value,
        )
        camera_hz: float = self.declare_parameter("camera_hz", 10.0).value
        show_viewer: bool = self.declare_parameter("viewer", False).value  # live MuJoCo window
        self._object_radius = list(self.declare_parameter("object_radius", [0.0, 0.0]).value)
        self._object_angle = list(self.declare_parameter("object_angle_deg", [0.0, 0.0]).value)
        # Arm pose at start (5 joints rad + gripper %), set before the object is placed:
        # all zeros = the backend's own start pose (arm stretched forward)
        self._start_pose = list(self.declare_parameter("start_pose", [0.0] * 6).value)
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")
        # The camera thread snapshots the sim state while the control loop steps it
        self._sim_lock = threading.Lock()
        self._camera_stop = threading.Event()
        self._camera_thread = None
        self._viewer = None

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")

        # Start by holding the current pose, so the arm doesn't jump on the first send_action.
        if self._passive:
            if not self._use_sim:
                self._robot.bus.disable_torque()
            self.get_logger().info("Passive mode: torque off, /joint_command ignored")

        # Real arm: model angle = motor angle - offset (zero in simulation)
        self._offsets = {name: 0.0 for name in JOINT_NAMES}
        if not self._use_sim and ZERO_OFFSETS_FILE.is_file():
            with open(ZERO_OFFSETS_FILE) as f:
                self._offsets.update({k: float(v) for k, v in (yaml.safe_load(f) or {}).items() if k in JOINT_NAMES})
            applied = ", ".join(f"{k} {v:+.3f}" for k, v in self._offsets.items() if v)
            self.get_logger().info(f"Zero offsets (rad): {applied}")
        elif not self._use_sim:
            self.get_logger().warn(f"No {ZERO_OFFSETS_FILE}: assuming the calibrated zero is the URDF zero pose. "
                                   "Measure it: ros2 run so101_driver measure_zero")

        # What is applied, for measure_zero (the file may have changed since we read it)
        self.declare_parameter("applied_zero_offsets", [self._offsets[n] for n in JOINT_NAMES[:5]])

        obs = self._model_obs()
        self._target = {f"{name}.pos": float(obs[f"{name}.pos"]) for name in JOINT_NAMES}

        self._pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self._cb_joint_command, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)

        if self._use_sim:
            if show_viewer:
                self._viewer = mujoco.viewer.launch_passive(self._robot.model, self._robot.data)
            self._setup_camera(camera_hz)
            self.create_service(Trigger, "/reset_object", self._srv_reset_object)
        else:
            self.get_logger().warn("Real arm: no camera published by the driver")

        self.get_logger().info("Driver node ready.")

    def _connect_arm(self, port: str = DEFAULT_PORT) -> SO101Sim | SO101Follower:
        """Creates and connects the backend chosen by use_sim (provided)."""
        if self._use_sim:
            kwargs = {}
            if self._scene:
                kwargs["scene_path"] = ASSETS_DIR / self._scene
            robot = SO101Sim(camera_name=self._camera, camera_width=self._camera_size[0],
                             camera_height=self._camera_size[1], **kwargs)
            robot.connect()
            if any(v != 0.0 for v in self._start_pose):
                self._set_start_pose(robot)
            self._place_object(robot)
            return robot

        if not CALIBRATION_FILE.is_file():
            raise RuntimeError(
                f"No calibration file at {CALIBRATION_FILE}: set CALIBRATION_FILE in docker/.env"
            )
        config = SO101FollowerConfig(
            port=port,
            id=CALIBRATION_FILE.stem,
            calibration_dir=CALIBRATION_FILE.parent,
            use_radians=True,
        )
        # The bus sometimes drops one reply (USB over WSL): the setup writes are not
        # retried by LeRobot, so retry the whole connection a few times.
        for attempt in range(1, 4):
            robot = SO101Follower(config)
            try:
                robot.connect(calibrate=False)
                break
            except ConnectionError as e:
                self.get_logger().warn(f"Connection attempt {attempt}/3 failed: {e}")
                if robot.bus.is_connected:
                    robot.bus.disconnect(disable_torque=False)
                if attempt == 3:
                    raise
                time.sleep(1.0)
        self._harden_serial(robot)
        if not robot.is_calibrated:
            robot.disconnect()
            raise RuntimeError(
                f"{CALIBRATION_FILE} does not match the motors: wrong file for this arm? "
                "Re-run lerobot-calibrate on the host."
            )
        return robot

    def _harden_serial(self, robot: SO101Follower):
        """Never block forever on the serial port. Over USB passed to WSL the link can
        stall: pyserial's flush() (tcdrain), called before every packet, then waits
        forever and the driver hangs with the motors still holding. Clear the input
        instead (what that call is for) and give writes a timeout: a stall becomes an
        error, handled like any bus error."""
        ph = robot.bus.port_handler
        ph.ser.write_timeout = 0.2
        ph.clearPort = ph.ser.reset_input_buffer

    def _set_start_pose(self, robot: SO101Sim):
        """Teleport the simulated arm to start_pose (before the object is placed: the
        default stretched-out pose stands where a tall object would go)."""
        for name, v in zip(JOINT_NAMES, self._start_pose):
            rad = robot._gripper_pct_to_rad(v) if name == "gripper" else v
            robot.data.qpos[robot._joint_qpos_adr[name]] = rad
            robot.data.ctrl[robot._actuator_id[name]] = rad
        robot.data.qvel[:] = 0
        mujoco.mj_forward(robot.model, robot.data)

    def _place_object(self, robot: SO101Sim):
        if self._object_radius[1] > 0:
            robot.randomize_object(radius_range=tuple(self._object_radius),
                                   angle_range_deg=tuple(self._object_angle))
        else:
            robot.randomize_object()

    def _srv_reset_object(self, request, response):
        with self._sim_lock:
            self._place_object(self._robot)
        response.success = True
        if mujoco.mj_name2id(self._robot.model, mujoco.mjtObj.mjOBJ_BODY, "pickup_object") >= 0:
            p = self._robot.get_object_position()
            response.message = f"object at ({p[0]:.3f}, {p[1]:.3f})"
        else:
            response.message = "scene objects reset"
        return response

    def _setup_camera(self, hz: float):
        """Image + CameraInfo at `hz`, and the camera pose as a static TF (simulation)."""
        self._cam_frame = f"{self._camera}_optical_frame"
        self._pub_image = self.create_publisher(Image, f"/{self._camera}/image_raw", 10)
        self._pub_cam_info = self.create_publisher(CameraInfo, f"/{self._camera}/camera_info", 10)

        K = self._robot.get_camera_intrinsics()
        self._cam_info = CameraInfo()
        self._cam_info.header.frame_id = self._cam_frame
        self._cam_info.width, self._cam_info.height = self._camera_size
        self._cam_info.distortion_model = "plumb_bob"
        self._cam_info.d = [0.0] * 5
        self._cam_info.k = K.flatten().tolist()
        self._cam_info.r = np.eye(3).flatten().tolist()
        self._cam_info.p = np.hstack([K, np.zeros((3, 1))]).flatten().tolist()

        # MuJoCo camera frame (x right, y up, looks along -z) -> optical frame (y down, looks along +z)
        T = self._robot.get_camera_extrinsics()
        R = T[:3, :3] @ np.diag([1.0, -1.0, -1.0])
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = WORLD_FRAME
        tf.child_frame_id = self._cam_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        q = Rotation.from_matrix(R).as_quat()  # x, y, z, w
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = map(float, q)
        self._tf_static = StaticTransformBroadcaster(self)
        self._tf_static.sendTransform(tf)

        # Software rendering (no GPU in the container) takes tens of ms: in the executor it
        # would starve the 50 Hz control loop, so it runs in its own thread, on a snapshot.
        self._camera_thread = threading.Thread(target=self._camera_loop, args=(hz,), daemon=True)
        self._camera_thread.start()
        w, h = self._camera_size
        self.get_logger().info(f"Camera: /{self._camera}/image_raw {w}x{h} @ {hz:.0f} Hz")

    def _camera_loop(self, hz: float):
        model = self._robot.model
        snapshot = mujoco.MjData(model)
        # The GL context belongs to the thread that creates the renderer: this one
        renderer = mujoco.Renderer(model, height=self._camera_size[1], width=self._camera_size[0])
        period = 1.0 / hz
        next_t = time.monotonic()
        while not self._camera_stop.wait(max(0.0, next_t - time.monotonic())):
            next_t += period
            with self._sim_lock:
                mujoco.mj_copyData(snapshot, model, self._robot.data)
            renderer.update_scene(snapshot, camera=self._camera)
            self._publish_image(renderer.render())
            next_t = max(next_t, time.monotonic())  # rendering slower than hz: don't pile up
        renderer.close()

    def _publish_image(self, img: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._cam_frame
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self._pub_image.publish(msg)
        self._cam_info.header.stamp = msg.header.stamp
        self._pub_cam_info.publish(self._cam_info)

    def destroy_node(self):
        self._camera_stop.set()
        if self._viewer is not None:
            self._viewer.close()
        if self._camera_thread is not None:
            self._camera_thread.join(timeout=2.0)
        robot = getattr(self, "_robot", None)
        if robot is not None and robot.is_connected:
            robot.disconnect()
        super().destroy_node()

    def _cb_joint_command(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("/joint_command: name and position lengths differ, ignored")
            return
        for name, pos in zip(msg.name, msg.position):
            if name not in JOINT_NAMES:
                self.get_logger().warn(f"/joint_command: unknown joint '{name}', ignored")
                continue
            self._target[f"{name}.pos"] = float(pos)

    def _model_obs(self):
        """Observation in model angles (motor angle - zero offset)."""
        obs = self._robot.get_observation()
        for name, off in self._offsets.items():
            if off and name != "gripper":
                obs[f"{name}.pos"] = float(obs[f"{name}.pos"]) - off
        return obs

    def _control_step(self):
        # State published right after the step: whoever reacts to it gets its command
        # applied at the very next step.
        # Real arm: the serial bus (USB over WSL) drops a reply now and then; skip that
        # step instead of dying. Only a lasting failure (power cut...) stops the driver.
        try:
            if not self._passive:
                with self._sim_lock:
                    self._robot.send_action({k: v + self._offsets.get(k.removesuffix(".pos"), 0.0)
                                             if k != "gripper.pos" else v for k, v in self._target.items()})
                    if self._viewer is not None and self._viewer.is_running():
                        self._viewer.sync()
            self._publish_joint_states()
            self._bus_failures = 0
        except (ConnectionError, serial.SerialException, OSError) as e:
            self._bus_failures = getattr(self, "_bus_failures", 0) + 1
            self.get_logger().warn(f"Motor bus: {e} (skipped step, {self._bus_failures} in a row)",
                                   throttle_duration_sec=1.0)
            if self._bus_failures >= MAX_BUS_FAILURES:
                self.get_logger().error("Motor bus lost (power? USB?): stopping the driver. The motors keep "
                                        "their last target until powered off.")
                raise

    def _publish_joint_states(self):
        with self._sim_lock:
            obs = self._model_obs()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [
            _gripper_pct_to_rad(obs[f"{name}.pos"]) if name == "gripper" else float(obs[f"{name}.pos"])
            for name in JOINT_NAMES
        ]
        self._pub_joint_states.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
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
