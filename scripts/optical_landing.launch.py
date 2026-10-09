"""Shared camera transport plus project optical plant/controller, no firmware."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root, shared = Path(os.environ["DRONE3_PROJECT_ROOT"]), Path(os.environ["CRAZYSIM_ROOT"])
    python = str(shared / ".venv/bin/python")
    sim = shared / "crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco"
    controller = ExecuteProcess(cmd=[python, str(root / "scripts/optical_landing_ros_node.py")], output="screen")
    camera = Node(package="aideck_ros2_bridge", executable="aideck_camera_node", output="screen",
                  parameters=[{"host": "127.0.0.1", "port": 5050, "topic": "/cf231/camera/image_raw", "frame_id": "optical_test"}])
    return LaunchDescription([
        ExecuteProcess(cmd=[python, str(sim / "crazysim_cpx.py"), "--camera-only"], output="screen"),
        TimerAction(period=1., actions=[ExecuteProcess(cmd=[python, str(root / "scripts/optical_landing_ros_plant.py")], output="screen")]),
        TimerAction(period=2., actions=[camera]), TimerAction(period=3., actions=[controller]),
        RegisterEventHandler(OnProcessExit(target_action=controller, on_exit=[EmitEvent(event=Shutdown(reason="optical mission finished"))]))])
