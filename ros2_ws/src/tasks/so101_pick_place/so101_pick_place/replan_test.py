#!/usr/bin/env python3
"""replan_test.py - simulation check of the replanning: triggers a pick & place and
teleports the cube (/reset_object) as soon as the arm starts its approach. Expected:
APPROACH -> CHECK_REACH -> APPROACH ... and SUCCESS with retries: 1.

    ros2 run so101_pick_place replan_test
"""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    n = rclpy.create_node("replan_test")
    events = []
    n.create_subscription(String, "/pick_place/state", lambda m: events.append(("state", m.data)), 10)
    n.create_subscription(String, "/pick_place/result", lambda m: events.append(("result", m.data)), 10)
    start, reset = n.create_client(Trigger, "/pick_and_place"), n.create_client(Trigger, "/reset_object")
    seen = []
    n.create_subscription(PoseStamped, "/object_position", lambda m: seen.append(time.time()), 10)
    start.wait_for_service()
    reset.wait_for_service()

    def call(client):
        f = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        return f.result().message if f.result() else "?"

    for _ in range(10):   # until the cube lands in the camera view
        print("reset:", call(reset), flush=True)
        t = time.time()
        while time.time() - t < 3.0:
            rclpy.spin_once(n, timeout_sec=0.05)
        if seen and time.time() - seen[-1] < 0.5:
            break
    start.call_async(Trigger.Request())
    moved, t_end = False, time.time() + 120
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if not moved and ("state", "APPROACH") in events:
            time.sleep(0.5)
            print("moved cube during APPROACH:", call(reset), flush=True)
            moved = True
        if any(k == "result" for k, _ in events):
            break
    print(" -> ".join(v for k, v in events if k == "state"))
    print([v for k, v in events if k == "result"])
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
