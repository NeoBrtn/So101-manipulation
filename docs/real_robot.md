# Running on the real SO-101

The same stack runs on the real arm: set `USE_SIM=false` and the driver swaps the
MuJoCo twin for LeRobot's `SO101Follower`. Every calibration file lives in
`ros2_ws/calibration/` (git-ignored, see its [README](../ros2_ws/calibration/README.md)).

## 1. Motor calibration (once per arm, on the host)

Calibrate with LeRobot (`lerobot-calibrate`), then point `CALIBRATION_FILE` in
`docker/.env` at the resulting JSON. It is mounted read-only in the control container.

## 2. Serial port and USB (WSL)

```bash
# Find the port: plug / unplug the arm while this runs
sudo udevadm monitor --subsystem-match=tty --property | grep --line-buffered -E "(DEVNAME=|ACTION=)"
```

Set `REAL_ROBOT_PORT` (usually `/dev/ttyACM0`). On WSL, attach the USB device first
(`usbipd attach --wsl --busid <id>`). The driver tolerates the occasional dropped reply
of USB over WSL.

## 3. Joint zero offsets (once per arm)

The LeRobot calibration puts each joint's zero in the middle of its range. The URDF
(and so the IK and the sim) has its own zero pose. Measure the difference:

```bash
# Torque off: the arm can be moved by hand
USE_SIM=false PASSIVE=true docker compose up -d --force-recreate control
```

Hold the arm in the URDF zero pose: upper arm vertical, forearm horizontal pointing
forward, wrist in line with the forearm, jaws in a vertical plane with the moving jaw
on top. Then:

```bash
docker compose exec control bash -lc "source install/setup.bash && ros2 run so101_driver measure_zero"
```

It writes `ros2_ws/calibration/arm_zero_offsets.yaml`. Restart the stack to apply it.

## 4. Camera

Any stream OpenCV can open works, e.g. a phone running DroidCam
(`http://<phone-ip>:4747/video`). Set `CAMERA_URL`.

```bash
ros2 run so101_driver calibrate_camera snapshot   --url $CAMERA_URL   # check the stream
ros2 run so101_driver calibrate_camera board                          # print the checkerboard
ros2 run so101_driver calibrate_camera intrinsics --url $CAMERA_URL   # lens: once per phone/resolution
ros2 run so101_driver calibrate_camera marker --marker-size 0.10      # print the ArUco marker
```

**Camera pose, live:** lay the ArUco marker (4×4, id 0) flat on the table with the
*top* of the print pointing away from the robot. Measure its centre in the robot frame
(x forward, y left), then set `MARKER_XY="x y"` and `MARKER_SIZE` (black square side,
in m). `camera_stream_node` estimates the camera pose from the marker on every frame
and filters it. It keeps the last pose while the arm hides the marker, and re-anchors
if the camera is moved.

Without a live marker, `calibrate_camera pose --marker-xy X Y` stores a fixed pose in
the calibration file instead.

## 5. Run

```bash
# docker/.env: USE_SIM=false, REAL_ROBOT_PORT, CALIBRATION_FILE, CAMERA_URL, MARKER_XY
docker compose up -d
ros2 service call /pick_and_place std_srvs/srv/Trigger      # the arm never moves on its own on the real setup
```

Test the IK alone first: `/go_to_target` moves the TCP to an XYZ and logs the error
measured from the joint encoders.

```bash
ros2 service call /go_to_target so101_interfaces/srv/GoToTarget \
  "{target_pose: {pose: {position: {x: 0.25, y: 0.0, z: 0.05}}}}"
```
