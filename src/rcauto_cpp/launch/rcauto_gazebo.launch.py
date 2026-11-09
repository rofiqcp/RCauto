"""Gazebo Harmonic RCauto 350x200 mm Ackermann, ROS 2 Humble.

Gazebo owns physical motion; robot_state_publisher owns link TF.
All hardware PWM remains disabled in this launch.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def select_mode_profile(context):
    """Overlay method-specific tuning without changing physical robot defaults."""
    mode = LaunchConfiguration("controller_mode").perform(context)
    if mode not in {"pid", "fuzzy", "ai"}:
        raise ValueError("controller_mode must be pid, fuzzy or ai")
    specified = LaunchConfiguration("mode_tuning_config").perform(context)
    if specified:
        if not Path(specified).is_file():
            raise FileNotFoundError(specified)
        return []
    directory = Path(get_package_share_directory("rcauto_cpp")) / "config"
    candidate = directory / ("gazebo_" + mode + ".yaml")
    profile = candidate if candidate.is_file() else directory / "gazebo_camera.yaml"
    return [SetLaunchConfiguration("mode_tuning_config", str(profile))]


def generate_launch_description():
    share = Path(get_package_share_directory("rcauto_cpp"))
    world = str(share / "sim" / "rcauto_track.sdf")
    description = str(share / "description" / "rcauto_350x200.urdf.xacro")
    bridge_config = str(share / "sim" / "gz_bridge.yaml")
    camera_tuning_config = str(share / "config" / "gazebo_camera.yaml")
    headless = LaunchConfiguration("headless")
    foxglove = LaunchConfiguration("foxglove_enabled")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description]),
        value_type=str,
    )
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "rcauto_cpp.launch.py")),
        condition=IfCondition(LaunchConfiguration("autonomy_enabled")),
        launch_arguments={
            "simulation": "false",
            "hardware_enabled": "false",
            "camera_source": "ros",
            "lane_style": "tape",
            "line_color": "white",
            "require_range": "false",
            "controller_mode": LaunchConfiguration("controller_mode"),
            "sim_steering_limit_rad": "0.70",
            "model_path": LaunchConfiguration("model_path"),
            "tuning_config": camera_tuning_config,
            "mode_tuning_config": LaunchConfiguration("mode_tuning_config"),
            "foxglove_enabled": foxglove,
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("foxglove_enabled", default_value="true"),
        DeclareLaunchArgument("controller_mode", default_value="pid"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("mode_tuning_config", default_value=""),
        OpaqueFunction(function=select_mode_profile),
        DeclareLaunchArgument("autonomy_enabled", default_value="true"),
        # Resolve model://rcauto_350x200 when loading world from installed package.
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                str(share / "sim"),
                os.pathsep,
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", world],
            output="screen",
            condition=IfCondition(headless),
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            output="screen",
            condition=UnlessCondition(headless),
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="rcauto_gz_bridge", output="screen",
            parameters=[{"config_file": bridge_config}],
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="rcauto_robot_state_publisher", output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="rcauto_cpp", executable="rcauto_gazebo_adapter",
            name="rcauto_gazebo_adapter", output="screen",
            parameters=[{"wheelbase_m": 0.244, "steering_limit_rad": 0.70}],
        ),
        autonomy,
    ])
