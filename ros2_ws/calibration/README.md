# Calibration data (local, not versioned)

Mounted in the control container as `/ros2_ws/calibration`. Everything here is specific
to one arm or one camera, so it is git-ignored.

| File | Written by | Used by |
|---|---|---|
| `arm_zero_offsets.yaml` | `ros2 run so101_driver measure_zero` | `so101_driver` (real arm) |
| `camera/<camera>.yaml` | `ros2 run so101_driver calibrate_camera intrinsics` / `pose` | `camera_stream` |
| `camera/*.png` | `calibrate_camera board` / `marker` / `snapshot` | printing, checks |

The LeRobot motor calibration (`lerobot-calibrate`) lives on the host and is mounted
separately (`CALIBRATION_FILE` in `docker/.env`). See [docs/real_robot.md](../../docs/real_robot.md).
