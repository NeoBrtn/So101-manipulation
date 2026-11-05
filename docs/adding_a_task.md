# Adding a task (project 3, …)

A task is one ROS 2 package in `ros2_ws/src/tasks/`, named `so101_<task>`. The
platform finds it by name. Nothing in `platform/` changes.

## 1. Package skeleton

```
ros2_ws/src/tasks/so101_mytask/
├── package.xml               depends on so101_manipulation (and so101_vision if it sees)
├── setup.py / setup.cfg      installs launch/*.launch.py and config/*.yaml
├── resource/so101_mytask
├── config/
│   ├── driver.yaml           simulation scene and camera for this task
│   └── mytask.yaml           the task's own parameters
├── launch/
│   ├── control.launch.py     required: the task node(s)
│   └── perception.launch.py  optional: detectors (run in the perception container)
└── so101_mytask/
    └── mytask_node.py
```

`so101_decap` is the smallest working example.

## 2. Simulation settings: `config/driver.yaml`

```yaml
so101_driver:
  ros__parameters:
    scene: my_scene.xml          # in sim/so101_sim/assets/so101/ ("" = scene_tek5.xml)
    camera_name: overhead_cam    # MJCF camera to publish as /<camera_name>/image_raw
    camera_width: 640
    camera_height: 480
    object_radius: [0.20, 0.30]  # random object placement (polar, around the base)
    object_angle_deg: [-40.0, 40.0]
    start_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 5 joints (rad) + gripper (%); zeros = default
```

## 3. The task node

Use the platform building blocks:

```python
from so101_manipulation.arm_interface import ArmInterface, spin_node
from so101_manipulation.kinematics import ArmKinematics
from so101_manipulation.trajectory import GRIPPER_OPEN, Waypoint


class MyTask(rclpy.node.Node):
    def __init__(self):
        super().__init__("mytask")
        self._kin = ArmKinematics()          # Pinocchio, from so101_description
        self._arm = ArmInterface(self)       # /joint_states in, /joint_command out
        self.create_timer(0.02, self._step)  # 50 Hz

    def _step(self):
        if not self._arm.ready:
            return
        if self._arm.streamer.q_cmd is None:       # first tick: start from where the arm is
            self._arm.hold_current()
            ik = self._kin.ik_top_down([0.25, 0.0, 0.05], self._arm.q)
            if ik.ok():
                self._arm.move([Waypoint("above", ik.q, GRIPPER_OPEN)])
        done = self._arm.stream()                  # interpolates and publishes
        ...


def main(args=None):
    spin_node(MyTask, args)
```

For perception, `so101_vision` gives you the camera model (K from `camera_info`,
pose from TF), pixel → plane projection, HSV blobs, and the occlusion-robust `Tracker`.

## 4. Run it

```bash
TASK=mytask docker compose up -d        # from docker/
# or, in the control container:
ros2 launch so101_bringup sim.launch.py task:=mytask
```

## 5. Tests

Put pure-Python logic (planning, geometry) in modules without ROS imports and test them
with pytest in `test/`. Add the package to `--packages-select` in
`.github/workflows/ci.yml` and `scripts/test.sh`.
