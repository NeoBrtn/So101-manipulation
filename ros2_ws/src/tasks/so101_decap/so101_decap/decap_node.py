#!/usr/bin/env python3
"""decap_node.py - phase 2 (design stage): unscrew a bottle cap.

Design: docs/phase2_decap.md. This node is the skeleton of the state machine: it is
wired into the platform (ArmInterface, ArmKinematics, the bringup task convention) but
the states are not implemented yet, so it stays in IDLE and never moves the arm.

    IDLE -> DETECT_BOTTLE -> CHECK_REACH -> APPROACH_BOTTLE -> GRASP_BOTTLE
         -> TRANSPORT_TO_FIXTURE -> INSERT -> LOCK -> GRASP_CAP -> UNSCREW_LOOP
         -> LIFT_CAP -> PLACE_CAP_IN_BOX -> UNLOCK -> REGRASP_BOTTLE
         -> PLACE_BOTTLE_IN_BOX -> HOME -> IDLE          (any failure -> RECOVER -> IDLE)
"""
from enum import Enum, auto

import rclpy
import rclpy.node
from std_msgs.msg import String

from so101_manipulation.arm_interface import ArmInterface, spin_node
from so101_manipulation.kinematics import ArmKinematics


class State(Enum):
    IDLE = auto()
    DETECT_BOTTLE = auto()
    CHECK_REACH = auto()
    APPROACH_BOTTLE = auto()
    GRASP_BOTTLE = auto()
    TRANSPORT_TO_FIXTURE = auto()
    INSERT = auto()
    LOCK = auto()
    GRASP_CAP = auto()
    UNSCREW_LOOP = auto()
    LIFT_CAP = auto()
    PLACE_CAP_IN_BOX = auto()
    UNLOCK = auto()
    REGRASP_BOTTLE = auto()
    PLACE_BOTTLE_IN_BOX = auto()
    HOME = auto()
    RECOVER = auto()


class DecapNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("decap")
        self.declare_parameters("", [
            ("min_reach", 0.12), ("max_reach", 0.34),
            ("move_threshold", 0.015), ("stale_timeout", 1.0),
            ("neck_grasp_height", 0.0), ("cap_grasp_height", 0.0),
            ("fixture_pose", [0.0, 0.0, 0.0]),
            ("cap_box_pose", [0.0, 0.0, 0.0]), ("bottle_box_pose", [0.0, 0.0, 0.0]),
            ("unscrew_angle", 1.2), ("unscrew_lift", 0.001), ("max_unscrew_steps", 8),
            ("max_retries", 3),
        ])
        self._kin = ArmKinematics()
        self._arm = ArmInterface(self)
        self._state = State.IDLE
        self._pub_state = self.create_publisher(String, "~/state", 10)
        self.create_timer(1.0, lambda: self._pub_state.publish(String(data=self._state.name)))
        self.get_logger().warn("Phase 2 (decap) is at the design stage: the state machine is not "
                               "implemented yet, the arm will not move. See docs/phase2_decap.md.")


def main(args=None):
    spin_node(DecapNode, args)


if __name__ == "__main__":
    main()
