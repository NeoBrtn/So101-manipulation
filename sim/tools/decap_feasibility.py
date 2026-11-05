#!/usr/bin/env python3
"""Feasibility check of the unscrewing motion in scene_decap.xml, without ROS.

Grasps the cap from above, then repeats strokes: rotate the gripper about the CAP AXIS
(not the wrist axis: the TCP is 7.8 mm off it) while rising with the thread, open,
rotate back, close. Ground truth (screw state) is only used to report.

    PYTHONPATH=sim:ros2_ws/src/platform/so101_manipulation python3 sim/tools/decap_feasibility.py
"""
import sys
from pathlib import Path

import numpy as np

from so101_manipulation.kinematics import ArmKinematics
from so101_sim import ASSETS_DIR, SO101Sim

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "ros2_ws/src/platform/so101_description/urdf/so101.urdf"
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DT = 0.01                      # s of sim per send_action (5 x 2 ms)

CAP_AXIS = np.array([0.20, 0.0])   # fixture centre (scene_decap.xml)
CAP_CENTER_Z = 0.0369          # scene_decap.xml
# Where the cap sits between the jaws, in the TCP frame (measured in sim: the TCP is on
# the fixed finger, the moving jaw closes from -x): the TCP is this far along +x from the
# cap axis, and this much lower than the cap centre (deeper in the jaws, or they slip).
GRASP_OFFSET_X = 0.018
GRASP_OFFSET_Z = float(__import__("os").environ.get("GRASP_DZ", "-0.008"))
# Push down while turning (like a person): the angled jaws otherwise squeeze the cap out
PRELOAD = float(__import__("os").environ.get("PRELOAD", "0.0"))
PITCH = 0.0027                 # m per turn
STROKE = np.radians(240)       # gripper rotation per stroke
MAX_STROKES = 3


def rot_z_about(axis_xy, theta, T):
    """Pose T rotated by theta about the vertical line through axis_xy."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    out = T.copy()
    p = T[:3, 3] - [axis_xy[0], axis_xy[1], 0]
    out[:3, 3] = R @ p + [axis_xy[0], axis_xy[1], 0]
    out[:3, :3] = R @ T[:3, :3]
    return out


class Arm:
    def __init__(self, sim):
        self.sim = sim
        obs = sim.get_observation()
        self.q = np.array([obs[f"{n}.pos"] for n in ARM])
        self.g = obs["gripper.pos"]

    def measured(self):
        obs = self.sim.get_observation()
        return np.array([obs[f"{n}.pos"] for n in ARM]), obs["gripper.pos"]

    def move(self, q, gripper=None, duration=1.0):
        q0, g0 = self.q.copy(), self.g
        g1 = g0 if gripper is None else gripper
        n = max(1, int(duration / DT))
        for i in range(1, n + 1):
            s = i / n
            s = 3 * s**2 - 2 * s**3
            self.q, self.g = q0 + (q - q0) * s, g0 + (g1 - g0) * s
            self.sim.send_action({**{f"{j}.pos": v for j, v in zip(ARM, self.q)}, "gripper.pos": self.g})
        for _ in range(int(0.3 / DT)):   # settle
            self.sim.send_action({**{f"{j}.pos": v for j, v in zip(ARM, self.q)}, "gripper.pos": self.g})


def main():
    kin = ArmKinematics(URDF)
    sim = SO101Sim(scene_path=ASSETS_DIR / "scene_decap.xml", camera_name="overhead_cam")
    sim.connect()
    if len(sys.argv) > 1:   # experiments: frictionloss damping armature of the cap's turn joint
        dof = sim.model.joint("bottle_cap_turn").dofadr[0]
        fl, dmp, arm_ = map(float, sys.argv[1:4])
        sim.model.dof_frictionloss[dof], sim.model.dof_damping[dof], sim.model.dof_armature[dof] = fl, dmp, arm_
        print(f"cap turn joint: frictionloss {fl}, damping {dmp}, armature {arm_}")
    arm = Arm(sim)
    cap = np.array([*CAP_AXIS, CAP_CENTER_Z])

    def grasp_for(a):
        x = np.array([np.cos(a), np.sin(a), 0.0])   # TCP x (jaw closing direction)
        return kin.ik_top_down(cap + GRASP_OFFSET_X * x + [0, 0, GRASP_OFFSET_Z], None, x[:2])

    # Start with the wrist near one end of its range, so a stroke can use most of it.
    grasp = None
    for a in np.radians(np.arange(0, 360, 15)):
        r = grasp_for(a)
        if r.ok() and (grasp is None or r.q[4] < grasp.q[4]):
            grasp = r
    print(f"grasp: IK error {grasp.position_error * 1000:.2f} mm, wrist_roll {grasp.q[4]:+.2f} rad")
    T_grasp = kin.fk(grasp.q)
    above = T_grasp[:3, 3] + [0, 0, 0.04]
    pre = kin.ik(above, T_grasp[:3, 2], T_grasp[:3, 0], grasp.q)

    arm.move(pre.q, 100.0, 2.5)
    arm.move(grasp.q, 100.0, 1.0)
    arm.move(grasp.q, 0.0, 1.2)
    q_meas, g_meas = arm.measured()
    print(f"closed on the cap: gripper {g_meas:.1f} % (0 = fully closed)")

    turned = 0.0
    for stroke in range(MAX_STROKES):
        # rotate (counter-clockwise from above) about the cap axis, rising with the thread
        steps = 24
        q = arm.q
        q_start = arm.q.copy()
        for i in range(1, steps + 1):
            th = STROKE * i / steps
            T = rot_z_about(CAP_AXIS, th, T_grasp)
            T[2, 3] += PITCH * (turned + th) / (2 * np.pi) - PRELOAD
            r = kin.ik(T[:3, 3], T[:3, 2], T[:3, 0], q)
            if not r.ok():
                print(f"  stroke {stroke + 1}: IK limit at {np.degrees(th):.0f} deg ({r.position_error * 1000:.1f} mm)")
                break
            q = r.q
            arm.move(q, 0.0, 0.15)
        else:
            th = STROKE
        turned += th
        turns, state = sim.get_screw_states()["bottle_cap"]
        released = state != "engaged"
        bottle_yaw = np.degrees(np.arctan2(sim.data.body("bottle").xmat[3], sim.data.body("bottle").xmat[0]))
        print(f"stroke {stroke + 1}: gripper turned {np.degrees(turned):.0f} deg, cap {turns * 360:.0f} deg "
              f"(slip {np.degrees(turned) - turns * 360:.0f} deg), released={released}, bottle yaw {bottle_yaw:+.1f}")
        if released:
            break
        # regrasp: open, rotate back, close
        arm.move(arm.q, 100.0, 0.8)
        T_back = T_grasp.copy()
        T_back[2, 3] += PITCH * turned / (2 * np.pi)
        r = kin.ik(T_back[:3, 3], T_back[:3, 2], T_back[:3, 0], q_start)   # not the short way round
        arm.move(r.q, 100.0, 1.5)
        arm.move(r.q, 0.0, 1.2)

    # lift the cap
    q_up = kin.ik_reach(kin.tcp_position(arm.q) + [0, 0, 0.06], arm.q)
    arm.move(q_up.q, 0.0, 1.5)
    cap_pos = sim.data.body("bottle_cap_free").xpos
    tcp = sim.data.site("gripperframe").xpos
    lifted = (sim.get_screw_states()["bottle_cap"][1] == "free" and cap_pos[2] > 0.07
              and np.linalg.norm(cap_pos - tcp) < 0.05)   # in the gripper
    print(f"lift: cap at {np.round(cap_pos, 3).tolist()} -> {'CAP REMOVED' if lifted else 'FAILED'}")
    return 0 if lifted else 1


if __name__ == "__main__":
    sys.exit(main())
