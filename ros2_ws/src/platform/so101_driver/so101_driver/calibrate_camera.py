#!/usr/bin/env python3
"""calibrate_camera.py - calibrate a real camera for the stack (lens, then pose in the world).

Run inside the control container; files go to /ros2_ws/calibration/camera (= ros2_ws/calibration/camera
on the host), where camera_stream_node reads them.

    ros2 run so101_driver calibrate_camera snapshot   --url URL           # 1. check the stream
    ros2 run so101_driver calibrate_camera board                          # 2. print the checkerboard
    ros2 run so101_driver calibrate_camera intrinsics --url URL           # 3. lens (K, distortion)
    ros2 run so101_driver calibrate_camera marker                         # 4. print the ArUco marker
    ros2 run so101_driver calibrate_camera pose --url URL --marker-xy 0.30 0.0   # 5. camera pose

`intrinsics` once per phone/resolution (the lens does not change when the camera moves).
`pose` is only needed without a live marker: camera_stream_node finds the camera pose
by itself from the marker (MARKER_XY in docker/.env), wherever the camera is.

Pose convention: the marker lies flat on the table, its centre at (x, y) in the robot
frame (world: x forward from the base, y to the left, z up), the TOP of the printed
marker pointing away from the robot (+x).
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from so101_driver.aruco_pose import ARUCO_DICT, camera_T_marker, make_detector, world_T_marker

CALIB_DIR = Path("/ros2_ws/calibration/camera")


def open_stream(url):
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        sys.exit(f"Cannot open {url}. Is the phone app streaming, same Wi-Fi, URL right?")
    return cap


def calib_path(camera):
    return CALIB_DIR / f"{camera}.yaml"


def save_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
    print(f"Saved {path}  (host: ros2_ws/{path.relative_to('/ros2_ws')})")


# -- subcommands ----------------------------------------------------------------------

def cmd_snapshot(a):
    cap = open_stream(a.url)
    for _ in range(5):          # skip buffered frames
        ok, frame = cap.read()
    if not ok:
        sys.exit("Stream opened but no frame received.")
    out = CALIB_DIR / f"{a.camera}_snapshot.png"
    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), frame)
    print(f"OK: {frame.shape[1]}x{frame.shape[0]} image saved to ros2_ws/calibration/camera/{out.name}")


def cmd_board(a):
    cols, rows = a.board                      # inner corners
    px = int(round(a.square / 0.0254 * 300))  # square size in pixels at 300 dpi
    img = np.full(((rows + 3) * px, (cols + 3) * px), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                y, x = (r + 1) * px, (c + 1) * px
                img[y:y + px, x:x + px] = 0
    out = CALIB_DIR / f"checkerboard_{cols}x{rows}_{a.square * 1000:.0f}mm.png"
    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), img)
    print(f"Saved ros2_ws/calibration/camera/{out.name}: print it at 300 dpi, 100 % scale (no 'fit to page'),\n"
          f"check that a square measures {a.square * 1000:.0f} mm, glue it on something flat and rigid.")


def cmd_marker(a):
    px = int(round(a.marker_size / 0.0254 * 300))
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), a.marker_id, px)
    border = px // 4
    img = cv2.copyMakeBorder(marker, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    cv2.putText(img, "TOP (away from robot)", (border, border // 2), cv2.FONT_HERSHEY_SIMPLEX,
                px / 600, 0, max(1, px // 300))
    out = CALIB_DIR / f"aruco_4x4_id{a.marker_id}_{a.marker_size * 1000:.0f}mm.png"
    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), img)
    print(f"Saved ros2_ws/calibration/camera/{out.name}: print at 300 dpi, 100 % scale, check the black square\n"
          f"measures {a.marker_size * 1000:.0f} mm (else pass the real size with --marker-size).")


def cmd_intrinsics(a):
    cols, rows = a.board
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * a.square
    cap = open_stream(a.url)
    obj_pts, img_pts, last_t, last_c, size = [], [], 0.0, None, None
    print(f"Show the {cols}x{rows} checkerboard to the camera: move it around the whole image, tilt it,\n"
          f"near and far. Collecting {a.views} views (one every ~{a.interval:.1f} s when it moved)...")
    t_end = time.time() + a.timeout
    while len(img_pts) < a.views and time.time() < t_end:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), cv2.CALIB_CB_FAST_CHECK)
        if not found or time.time() - last_t < a.interval:
            continue
        c = corners.reshape(-1, 2).mean(0)
        if last_c is not None and np.linalg.norm(c - last_c) < 0.05 * size[0]:
            continue  # too similar to the previous view
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        obj_pts.append(objp)
        img_pts.append(corners)
        last_t, last_c = time.time(), c
        print(f"  view {len(img_pts)}/{a.views}")
    if len(img_pts) < 8:
        sys.exit(f"Only {len(img_pts)} views: not enough. Check --board (INNER corners) and the lighting.")
    rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    print(f"Reprojection error: {rms:.2f} px  ({'good' if rms < 0.6 else 'ok' if rms < 1.2 else 'poor: redo it'})")
    print(f"fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
    save_yaml(calib_path(a.camera), {
        "image_width": int(size[0]), "image_height": int(size[1]),
        "camera_matrix": K.flatten().tolist(), "distortion": D.flatten()[:5].tolist(),
        "reprojection_error_px": float(rms),
    })
    print("Camera pose not calibrated yet (or reset): run the `pose` command next.")


def cmd_pose(a):
    path = calib_path(a.camera)
    calib = yaml.safe_load(open(path)) if path.is_file() else None
    if calib is None:
        sys.exit(f"No {path}: run `intrinsics` first.")
    K = np.array(calib["camera_matrix"], dtype=float).reshape(3, 3)
    D = np.array(calib["distortion"], dtype=float)
    detector = make_detector()

    cap = open_stream(a.url)
    rots, trans, t_end = [], [], time.time() + 20.0
    while len(trans) < 30 and time.time() < t_end:
        ok, frame = cap.read()
        if not ok:
            continue
        T = camera_T_marker(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), K, D, a.marker_id, a.marker_size)
        if T is not None:
            rots.append(Rotation.from_matrix(T[:3, :3]))
            trans.append(T[:3, 3])
    if len(trans) < 5:
        sys.exit(f"Marker id {a.marker_id} seen in only {len(trans)} frames: is it in view, flat, well lit?")

    cam_T_marker = np.eye(4)
    cam_T_marker[:3, :3] = Rotation.concatenate(rots).mean().as_matrix()
    cam_T_marker[:3, 3] = np.median(trans, axis=0)
    world_T_cam = world_T_marker(a.marker_xy, a.marker_z) @ np.linalg.inv(cam_T_marker)

    p = world_T_cam[:3, 3]
    look = world_T_cam[:3, 2]   # optical axis
    print(f"Camera at x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} m, looking along {np.round(look, 2).tolist()}"
          f" ({np.degrees(np.arccos(np.clip(-look[2], -1, 1))):.0f}° from straight down)")
    print(f"Marker distance spread over {len(trans)} frames: {np.ptp(np.linalg.norm(trans, axis=1)) * 1000:.1f} mm")
    if p[2] <= 0:
        print("WARNING: camera below the table: wrong --marker-size or marker orientation?")
    calib["world_T_camera"] = world_T_cam.flatten().tolist()
    save_yaml(path, calib)
    print("Restart the container (or the camera_stream node) to publish the new pose.")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp, url=True):
        if url:
            sp.add_argument("--url", required=True, help="stream URL, e.g. http://192.168.1.42:4747/video")
        sp.add_argument("--camera", default="overhead_cam", help="camera name (topic and file name)")

    def board(sp):
        sp.add_argument("--board", type=int, nargs=2, default=[9, 6], metavar=("COLS", "ROWS"),
                        help="INNER corners of the checkerboard (default 9 6)")
        sp.add_argument("--square", type=float, default=0.025, help="square size in m (default 0.025)")

    def marker(sp):
        sp.add_argument("--marker-id", type=int, default=0)
        sp.add_argument("--marker-size", type=float, default=0.10, help="black square side in m (default 0.10)")

    sp = sub.add_parser("snapshot", help="grab one frame, check the stream works")

    common(sp)
    sp = sub.add_parser("board", help="write a printable checkerboard")
    common(sp, url=False)
    board(sp)
    sp = sub.add_parser("marker", help="write a printable ArUco marker")
    common(sp, url=False)
    marker(sp)
    sp = sub.add_parser("intrinsics", help="lens calibration with the checkerboard")
    common(sp)
    board(sp)
    sp.add_argument("--views", type=int, default=20)
    sp.add_argument("--interval", type=float, default=0.8, help="min seconds between views")
    sp.add_argument("--timeout", type=float, default=180.0)
    sp = sub.add_parser("pose", help="camera pose from the marker on the table")
    common(sp)
    marker(sp)
    sp.add_argument("--marker-xy", type=float, nargs=2, required=True, metavar=("X", "Y"),
                    help="marker centre in the robot frame (m)")
    sp.add_argument("--marker-z", type=float, default=0.0, help="marker height (m), 0 = on the table")

    a = p.parse_args(argv)
    {"snapshot": cmd_snapshot, "board": cmd_board, "marker": cmd_marker,
     "intrinsics": cmd_intrinsics, "pose": cmd_pose}[a.cmd](a)


if __name__ == "__main__":
    main()
