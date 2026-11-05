#!/usr/bin/env python3
"""Generates sim/so101_sim/assets/so101/scene_decap.xml (phase 2: bottle decapping).

Robot on an 18 cm riser (the table is at z = -RISER in the robot frame), the bottle
standing in the printed fixture (hardware/bottle_fixture/bottle_fixture.scad, same
dimensions), a cap box and a bottle box on the table.

The cap is a screw (see SO101Sim's screw caps): a hinge + slide pair coupled by the
thread pitch, with friction for the tightness. After `turns` turns it is released and
becomes a free body.

    python3 sim/tools/make_decap_scene.py
"""
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parents[1] / "so101_sim" / "assets" / "so101" / "scene_decap.xml"

RISER = 0.18                    # m, robot base above the table
FIXTURE_XY = (0.20, 0.0)        # fixture centre, robot frame
CAP_BOX_XY = (0.12, -0.22)
BOTTLE_BOX_XY = (0.02, 0.25)

# Fixture (mm in the SCAD file)
BOTTLE_D = 0.060
CLEARANCE = 0.001
WALL = 0.003
FLOOR_T = 0.003
CUP_H = 0.035
FEET = 5
RIB_W, RIB_R_IN, RIB_H = 0.003, 0.014, 0.009   # sim: one box per rib (SCAD: 4 -> 10 mm wedge)
FLANGE_D, FLANGE_T = 0.110, 0.004

# Bottle (Auchan 50 cl, measured from photos): 220 mm tall
FOOT_R, FOOT_R_IN, FOOT_R_OUT = 0.007, 0.017, 0.023   # radial capsule feet, between the ribs
BODY_Z = (0.014, 0.100)               # cylinder Ø60
SHOULDER = [(0.100, 0.115, 0.0285), (0.115, 0.130, 0.026), (0.130, 0.145, 0.023),
            (0.145, 0.160, 0.0195), (0.160, 0.175, 0.0165), (0.175, 0.190, 0.0140)]  # z0, z1, radius
NECK = (0.190, 0.205, 0.0125)
CAP_Z, CAP_R, CAP_H = 0.205, 0.014, 0.017    # cap bottom, radius, height (incl. tamper band)
WATER_KG, WATER_H = 0.100, 0.040

# Thread: PCO 1881 (28 mm carbonated-drink finish)
PITCH = 0.0027        # m per turn
TURNS = 1.25          # turns until the cap is free
TIGHTNESS = 0.03      # N m of friction torque (loosened cap)


def f(*v):
    return " ".join(f"{x:.5g}" for x in v)


def fixture_xml():
    inner_r = BOTTLE_D / 2 + CLEARANCE
    z0 = -RISER
    g = [f'<geom name="fixture_flange" type="cylinder" size="{f(FLANGE_D / 2, FLANGE_T / 2)}" '
         f'pos="{f(0, 0, z0 + FLANGE_T / 2)}" material="fixture"/>',
         f'<geom name="fixture_floor" type="cylinder" size="{f(inner_r + WALL, FLOOR_T / 2)}" '
         f'pos="{f(0, 0, z0 + FLOOR_T / 2)}" material="fixture"/>']
    # wall: ring of boxes
    n = 32
    seg = 2 * (inner_r + WALL / 2) * np.tan(np.pi / n) * 1.05
    for i in range(n):
        a = 2 * np.pi * i / n
        r = inner_r + WALL / 2
        g.append(f'<geom type="box" size="{f(WALL / 2, seg / 2, CUP_H / 2)}" '
                 f'pos="{f(r * np.cos(a), r * np.sin(a), z0 + FLOOR_T + CUP_H / 2)}" '
                 f'euler="0 0 {a:.5f}" material="fixture"/>')
    # ribs
    length = inner_r - RIB_R_IN
    for i in range(FEET):
        a = 2 * np.pi * i / FEET
        r = RIB_R_IN + length / 2
        g.append(f'<geom name="fixture_rib{i}" type="box" size="{f(length / 2, RIB_W / 2, RIB_H / 2)}" '
                 f'pos="{f(r * np.cos(a), r * np.sin(a), z0 + FLOOR_T + RIB_H / 2)}" '
                 f'euler="0 0 {a:.5f}" material="fixture"/>')
    body = "\n            ".join(g)
    return f'''<body name="fixture" pos="{f(*FIXTURE_XY, 0)}">
            {body}
        </body>'''


def bottle_xml():
    z0 = -RISER + FLOOR_T
    g = []
    for i in range(FEET):   # feet between the ribs
        a = 2 * np.pi * (i + 0.5) / FEET
        c, s_ = np.cos(a), np.sin(a)
        g.append(f'<geom class="bottle" type="capsule" size="{FOOT_R}" fromto="'
                 f'{f(FOOT_R_IN * c, FOOT_R_IN * s_, FOOT_R, FOOT_R_OUT * c, FOOT_R_OUT * s_, FOOT_R)}"/>')
    body_half = (BODY_Z[1] - BODY_Z[0]) / 2
    g.append(f'<geom name="bottle_body" class="bottle" type="cylinder" size="{f(BOTTLE_D / 2, body_half)}" '
             f'pos="{f(0, 0, sum(BODY_Z) / 2)}" mass="0.012"/>')
    # ~100 ml of water: steadier bottle (in the sim as for real), no contacts
    g.append(f'<geom name="bottle_water" type="cylinder" size="{f(BOTTLE_D / 2 - 0.002, WATER_H / 2)}" '
             f'pos="{f(0, 0, BODY_Z[0] + WATER_H / 2)}" mass="{WATER_KG}" contype="0" conaffinity="0" '
             f'rgba="0.6 0.8 1 0.25"/>')
    for z_a, z_b, r in SHOULDER:
        g.append(f'<geom class="bottle" type="cylinder" size="{f(r, (z_b - z_a) / 2)}" '
                 f'pos="{f(0, 0, (z_a + z_b) / 2)}"/>')
    g.append(f'<geom name="bottle_neck" class="bottle" type="cylinder" size="{f(NECK[2], (NECK[1] - NECK[0]) / 2)}" '
             f'pos="{f(0, 0, (NECK[0] + NECK[1]) / 2)}" friction="1.5 0.05 0.001"/>')
    cap_geom = (f'type="cylinder" size="{f(CAP_R, CAP_H / 2)}" material="bottle_cap" mass="0.003" '
                f'friction="1.5 0.05 0.001" condim="4" solref="0.004 1" solimp="0.99 0.999 0.0005"')
    body = "\n            ".join(g)
    return f'''<body name="bottle" pos="{f(*FIXTURE_XY, z0)}">
            <freejoint name="bottle_free"/>
            {body}
            <!-- Screwed cap: turning it (+z, counter-clockwise seen from above) lifts it by the
                 thread pitch; frictionloss = how tight it is. Released after {TURNS} turns. -->
            <body name="bottle_cap" pos="{f(0, 0, CAP_Z + CAP_H / 2)}">
                <joint name="bottle_cap_turn" type="hinge" axis="0 0 1" range="{f(-0.05, TURNS * 2 * np.pi + 0.5)}"
                    frictionloss="{TIGHTNESS}" damping="0.002" armature="0.0001"/>
                <joint name="bottle_cap_lift" type="slide" axis="0 0 1" range="{f(-0.001, 0.05)}" armature="0.0001"/>
                <geom name="bottle_cap_geom" {cap_geom}/>
            </body>
        </body>
        <!-- The cap once released (parked out of the scene until then) -->
        <body name="bottle_cap_free" pos="0 0 2" gravcomp="1">
            <freejoint name="bottle_cap_free"/>
            <!-- heavier than the real 3 g: a light free body is squeezed through by the jaws -->
            <geom name="bottle_cap_free_geom" {cap_geom.replace('mass="0.003"', 'mass="0.03"')}/>
        </body>'''


def open_box_xml(name, xy, size, height):
    sx, sy = size
    t = 0.003
    z0 = -RISER
    walls = [((0, 0, t / 2), (sx / 2, sy / 2, t / 2)),
             ((sx / 2, 0, height / 2), (t / 2, sy / 2, height / 2)),
             ((-sx / 2, 0, height / 2), (t / 2, sy / 2, height / 2)),
             ((0, sy / 2, height / 2), (sx / 2, t / 2, height / 2)),
             ((0, -sy / 2, height / 2), (sx / 2, t / 2, height / 2))]
    g = "\n            ".join(f'<geom type="box" size="{f(*s)}" pos="{f(p[0], p[1], z0 + p[2])}" material="{name}"/>'
                              for p, s in walls)
    return f'''<body name="{name}" pos="{f(*xy, 0)}">
            {g}
        </body>'''


GRIP_SOLVER = 'solref="0.004 1" solimp="0.99 0.999 0.001"'   # stiff: holds the part where it was grasped


def main():
    k = PITCH / (2 * np.pi)
    xml = f'''<mujoco model="so101_decap">
    <!-- GENERATED by sim/tools/make_decap_scene.py: edit the script, not this file. -->
    <include file="so101_tek5.xml"/>
    <custom>
        <!-- Screw caps handled by SO101Sim: turns before release -->
        <numeric name="screw:bottle_cap" data="{TURNS}"/>
        <!-- Grip assist: stands in for the grip pads (see SO101Sim) -->
        <numeric name="grip_assist:bottle_cap" data="1"/>
        <numeric name="grip_assist:bottle_cap_free" data="1"/>
        <numeric name="grip_assist:bottle" data="1"/>
    </custom>
    <visual>
        <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
        <global azimuth="150" elevation="-25" offwidth="1280" offheight="720"/>
    </visual>
    <default>
        <default class="bottle">
            <geom material="bottle" mass="0.001" friction="1.0 0.01 0.001" condim="4"
                solref="0.01 1" solimp="0.95 0.99 0.001"/>
        </default>
    </default>
    <asset>
        <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.25 0.25 0.28"
            rgb2="0.18 0.18 0.20" markrgb="0.6 0.6 0.6" width="300" height="300"/>
        <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.1"/>
        <material name="riser" rgba="0.45 0.35 0.25 1"/>
        <material name="fixture" rgba="0.25 0.45 0.75 1"/>
        <material name="bottle" rgba="0.85 0.9 0.95 0.35" specular="0.6" shininess="0.8"/>
        <material name="bottle_cap" rgba="0.05 0.45 0.15 1"/>
        <material name="cap_box" rgba="0.35 0.20 0.12 1"/>
        <material name="bottle_box" rgba="0.20 0.30 0.40 1"/>
    </asset>
    <equality>
        <!-- thread: lift = pitch / (2 pi) * turn -->
        <joint name="bottle_cap_thread" joint1="bottle_cap_lift" joint2="bottle_cap_turn" polycoef="0 {k:.6g} 0 0 0"
            solref="0.005 1"/>
        <weld name="grip_bottle_cap" body1="moving_jaw_so101_v1" body2="bottle_cap" active="false"
            {GRIP_SOLVER}/>
        <weld name="grip_bottle_cap_free" body1="moving_jaw_so101_v1" body2="bottle_cap_free" active="false"
            {GRIP_SOLVER}/>
        <weld name="grip_bottle" body1="moving_jaw_so101_v1" body2="bottle" active="false"
            {GRIP_SOLVER}/>
    </equality>
    <worldbody>
        <light pos="0 0 2.5" dir="0 0 -1" directional="true"/>
        <light pos="0.5 0.5 1.5" dir="-0.3 -0.3 -1" diffuse="0.4 0.4 0.4"/>
        <geom name="floor" type="plane" size="0 0 0.05" pos="0 0 {-RISER}" material="groundplane"/>
        <!-- {RISER * 100:.0f} cm riser under the robot base -->
        <geom name="riser" type="box" size="0.07 0.06 {RISER / 2}" pos="0.0 0 {-RISER / 2 - 0.0005}" material="riser"/>
        {fixture_xml()}
        {bottle_xml()}
        {open_box_xml("cap_box", CAP_BOX_XY, (0.08, 0.08), 0.04)}
        {open_box_xml("bottle_box", BOTTLE_BOX_XY, (0.12, 0.12), 0.05)}
        <camera name="external_cam" pos="0.45 0.45 0.37" xyaxes="-0.707 0.707 0 -0.354 -0.354 0.866" fovy="52"/>
        <camera name="overhead_cam" pos="0.15 0.0 0.62" xyaxes="0 -1 0 1 0 0" fovy="62"/>
        <camera name="side_cam" pos="0.20 -0.55 0.05" xyaxes="1 0 0 0 0.25 0.97" fovy="55"/>
    </worldbody>
</mujoco>
'''
    OUT.write_text(xml)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
