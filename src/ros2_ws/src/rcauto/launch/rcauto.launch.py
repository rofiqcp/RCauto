"""Use: ros2 launch rcauto rcauto.launch.py simulation:=true camera_source:=ros
WARNING: simulation:=true must NEVER be paired with hardware_enabled:=true.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def validate_configuration(context):
    simulation = LaunchConfiguration("simulation").perform(context).lower() in ("1", "true", "yes")
    hardware = LaunchConfiguration("hardware_enabled").perform(context).lower() in ("1", "true", "yes")
    if simulation and hardware:
        raise RuntimeError("Simulation input and LIVE PWM are mutually exclusive")
    return []


def generate_launch_description():
    config = str(Path(get_package_share_directory("rcauto")) / "config" / "robot.yaml")
    simulation = LaunchConfiguration("simulation")
    live_hardware = LaunchConfiguration("hardware_enabled")
    camera_source = LaunchConfiguration("camera_source")
    hailo = LaunchConfiguration("use_hailo")
    mode = LaunchConfiguration("mode")
    model = LaunchConfiguration("model_path")
    return LaunchDescription([
        DeclareLaunchArgument("simulation", default_value="false"),
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("camera_source", default_value="usb"),
        DeclareLaunchArgument("lane_style", default_value="tape"),
        DeclareLaunchArgument("use_hailo", default_value="false"),
        DeclareLaunchArgument("mode", default_value="pid"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("foxglove_enabled", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8767"),
        DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
        OpaqueFunction(function=validate_configuration),
        Node(package="rcauto", executable="vision", name="rcauto_vision", output="screen",
             parameters=[config, {"source": camera_source, "lane_style": LaunchConfiguration("lane_style")}]),
        Node(package="rcauto", executable="autonomy", name="rcauto_autonomy", output="screen",
             parameters=[config, {"mode": mode, "model_path": model}]),
        Node(package="rcauto", executable="actuator", name="rcauto_actuator", output="screen",
             parameters=[config, {"hardware_enabled": ParameterValue(live_hardware, value_type=bool)}]),
        Node(package="rcauto", executable="simulator", name="rcauto_simulator",
             output="screen", condition=IfCondition(simulation)),
        Node(package="rcauto", executable="hailo_bridge", name="rcauto_hailo_bridge",
             output="screen", condition=IfCondition(hailo)),
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
             output="screen", parameters=[{"port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
                                            "address": LaunchConfiguration("foxglove_address")}],
             condition=IfCondition(LaunchConfiguration("foxglove_enabled"))),
    ])
