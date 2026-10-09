"""Reuse the pinned team server and camera bridge; add project yaw node."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(os.environ["DRONE3_PROJECT_ROOT"])
    shared = Path(os.environ["CRAZYSIM_ROOT"])
    out = Path(os.environ["DRONE3_EXPERIMENT_DIR"])
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("crazyflie")) / "launch/launch.py")),
        launch_arguments={"backend": "cflib", "gui": "False", "mocap": "False", "teleop": "False",
                          "crazyflies_yaml_file": str(out / "crazyflies.yaml")}.items())
    camera = Node(package="aideck_ros2_bridge", executable="aideck_camera_node", name="aideck_camera_node", output="screen",
                  parameters=[{"host": "127.0.0.1", "port": 5050, "topic": "/cf231/camera/image_raw", "frame_id": "cf231_camera"}])
    control = ExecuteProcess(cmd=[str(shared / ".venv/bin/python"), str(root / "scripts/yaw_control_node.py"), "--output", str(out)], output="screen")
    return LaunchDescription([
        ExecuteProcess(cmd=["bash", str(root / "scripts/yaw_sitl.sh")], output="screen"),
        TimerAction(period=4., actions=[camera]), TimerAction(period=7., actions=[server]),
        TimerAction(period=10., actions=[control]),
        RegisterEventHandler(OnProcessExit(target_action=control, on_exit=[EmitEvent(event=Shutdown(reason="yaw node exited"))]))])
