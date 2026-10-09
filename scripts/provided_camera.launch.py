"""Camera-only transport check using pinned CPX and ROS bridge, no aircraft."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(os.environ["DRONE3_PROJECT_ROOT"])
    shared = Path(os.environ["CRAZYSIM_ROOT"])
    out = Path(os.environ["DRONE3_EXPERIMENT_DIR"])
    python = str(shared / ".venv/bin/python")
    sim = shared / "crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco"
    check = ExecuteProcess(cmd=[python, str(root / "scripts/provided_camera_node.py"), "--output", str(out)], output="screen")
    camera = Node(package="aideck_ros2_bridge", executable="aideck_camera_node", output="screen",
                  parameters=[{"host": "127.0.0.1", "port": 5050, "topic": "/cf231/camera/image_raw", "frame_id": "optical_preview_camera"}])
    return LaunchDescription([
        ExecuteProcess(cmd=[python, str(sim / "crazysim_cpx.py"), "--camera-only"], output="screen"),
        TimerAction(period=1., actions=[ExecuteProcess(cmd=[python, str(root / "scripts/provided_camera_stream.py")], output="screen")]),
        TimerAction(period=2., actions=[camera]), TimerAction(period=3., actions=[check]),
        RegisterEventHandler(OnProcessExit(target_action=check, on_exit=[EmitEvent(event=Shutdown(reason="provided camera check finished"))]))])
