#!/usr/bin/env python3
"""measure_zero.py - measure the real arm's zero offsets against the URDF zero pose.

The real arm's calibrated zero is the middle of each joint's recorded range, which is
not exactly the pose the URDF (IK, simulation) calls zero. This measures the difference.

    1. Start the real arm in passive mode (torque off):
         USE_SIM=false PASSIVE=true docker compose up -d --force-recreate control
    2. Hold the arm in the URDF zero pose: upper arm vertical, forearm horizontal
       pointing straight forward, wrist straight in line with the forearm, gripper
       jaws in a vertical plane, the moving jaw on top (opening upward).
    3. ros2 run so101_driver measure_zero
    4. Restart the stack: the driver applies the offsets.

Re-running it is fine: it asks the driver which offsets it applies (it reads the file
only at startup) and corrects those.
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState

from so101_driver.driver_node import JOINT_NAMES, ZERO_OFFSETS_FILE

ARM = JOINT_NAMES[:5]


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("measure_zero")
    samples = []
    node.create_subscription(JointState, "/joint_states",
                             lambda m: samples.append(dict(zip(m.name, m.position))), 10)
    print("Hold the arm still in the zero pose... measuring for 2 s")
    t_end = time.time() + 2.0
    while time.time() < t_end or not samples:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not samples and time.time() > t_end + 5:
            sys.exit("No /joint_states: is the real arm stack running (passive mode)?")
    q = {n: float(np.median([s[n] for s in samples])) for n in ARM}
    spread = {n: float(np.ptp([s[n] for s in samples])) for n in ARM}

    # /joint_states = motor angle - the offsets the driver applies: ask it which
    cli = node.create_client(GetParameters, "/so101_driver/get_parameters")
    if not cli.wait_for_service(timeout_sec=5.0):
        sys.exit("Driver not found (/so101_driver): is the real arm stack running?")
    fut = cli.call_async(GetParameters.Request(names=["applied_zero_offsets"]))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    values = fut.result().values if fut.result() else []
    applied = list(values[0].double_array_value) if values and values[0].double_array_value else [0.0] * 5
    new = {n: round(applied[i] + q[n], 4) for i, n in enumerate(ARM)}

    print(f"{'joint':14s} {'reads (deg)':>12s} {'new offset (deg)':>17s}")
    for n in ARM:
        print(f"{n:14s} {np.degrees(q[n]):12.1f} {np.degrees(new[n]):17.1f}"
              + ("   <- moved while measuring?" if spread[n] > 0.02 else ""))
    if abs(np.degrees(q["wrist_roll"])) > 45:
        print("wrist_roll is far from zero: are the jaws in a vertical plane, moving jaw on top?")
    with open(ZERO_OFFSETS_FILE, "w") as f:
        f.write("# Real arm zero offsets (rad): motor angle at the URDF zero pose.\n"
                "# Written by `ros2 run so101_driver measure_zero`; applied by the driver.\n")
        yaml.safe_dump(new, f, sort_keys=False)
    print(f"Saved {ZERO_OFFSETS_FILE} (host: ros2_ws/calibration/{ZERO_OFFSETS_FILE.name}).")
    print("Restart the stack to apply it.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
