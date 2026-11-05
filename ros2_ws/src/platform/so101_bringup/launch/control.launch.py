"""control.launch.py - platform + the control side of a task (control container).

    ros2 launch so101_bringup control.launch.py task:=pick_place
    ros2 launch so101_bringup control.launch.py task:=pick_place use_sim:=false \
        camera_url:=http://192.168.1.42:4747/video marker_xy:="0.17 0"

Starts:
  - so101_driver (sim or real arm), with the task's simulation settings from
    share/so101_<task>/config/driver.yaml (scene, camera, object placement)
  - camera_stream (real arm only, if camera_url is set): a network camera published
    like the simulated one, posed live from an ArUco marker (marker_xy)
  - motion_server (FK pose, /go_to_target, /set_joint_positions)
  - share/so101_<task>/launch/control.launch.py (the task's own nodes)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from so101_bringup.tasks import arg, include_task_launch, task_driver_params, task_share


def marker_params(context):
    """marker_xy "0.30 0.0" (m, robot frame) -> live camera pose from the ArUco marker."""
    xy = arg(context, "marker_xy").replace(",", " ").split()
    if not xy:
        return {}
    if len(xy) != 2:
        raise ValueError(f"marker_xy: expected 'x y', got {xy}")
    return {"marker_xy": [float(v) for v in xy], "marker_size": float(arg(context, "marker_size"))}


def launch_setup(context):
    task = arg(context, "task")
    share = task_share(task)
    use_sim = arg(context, "use_sim").lower() == "true"
    viewer = arg(context, "viewer").lower() == "true"
    driver_params = task_driver_params(share)
    camera = driver_params.get("camera_name", "external_cam")

    actions = [Node(
        package="so101_driver", executable="driver", name="so101_driver", output="screen",
        # The MuJoCo window needs real OpenGL (GLFW): the whole process then renders with it
        additional_env={"MUJOCO_GL": "glfw"} if viewer else {},
        parameters=[driver_params, {
            "use_sim": use_sim,
            "port": arg(context, "port"),
            "passive": arg(context, "passive").lower() == "true",
            "viewer": viewer,
        }],
    )]
    camera_url = arg(context, "camera_url")
    if camera_url and not use_sim:
        actions.append(Node(
            package="so101_driver", executable="camera_stream", name="camera_stream", output="screen",
            parameters=[{"url": camera_url, "camera_name": camera,
                         "calibration": f"/ros2_ws/calibration/camera/{camera}.yaml",
                         **marker_params(context)}],
        ))
    actions.append(Node(package="so101_manipulation", executable="motion_server", output="screen"))
    actions += include_task_launch(share, "control.launch.py", {"use_sim": str(use_sim).lower()})
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("task", default_value="pick_place", description="Task package: so101_<task>"),
        DeclareLaunchArgument("use_sim", default_value="true", description="MuJoCo sim or real arm"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0", description="Real arm serial port"),
        DeclareLaunchArgument("passive", default_value="false",
                              description="Real arm: torque off, read-only (move it by hand)"),
        DeclareLaunchArgument("viewer", default_value="false", description="Sim: live MuJoCo window (X11)"),
        DeclareLaunchArgument("camera_url", default_value="",
                              description="Real arm: network camera stream (e.g. DroidCam)"),
        DeclareLaunchArgument("marker_xy", default_value="",
                              description="Real camera: ArUco marker centre 'x y' (m, robot frame)"),
        DeclareLaunchArgument("marker_size", default_value="0.10", description="ArUco black square side (m)"),
        OpaqueFunction(function=launch_setup),
    ])
