"""C++-only ROS nodes. Simulated data and physical PWM must never coexist."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def validate(context):
    simulation = LaunchConfiguration("simulation").perform(context).lower() in ("1", "true")
    hardware = LaunchConfiguration("hardware_enabled").perform(context).lower() in ("1", "true")
    if simulation and hardware:
        raise RuntimeError("Simulation is forbidden with live actuator PWM")
    if hardware and LaunchConfiguration("live_ack").perform(context) != "CONFIRM_WHEELS_UP_EXTERNAL_ESTOP":
        raise RuntimeError("Live PWM requires explicit wheels-up and external E-stop acknowledgement")
    return []


def generate_launch_description():
    config = str(Path(get_package_share_directory("rcauto_cpp")) / "config" / "robot.yaml")
    sim = LaunchConfiguration("simulation")
    return LaunchDescription([
        DeclareLaunchArgument("simulation", default_value="false"),
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("require_range", default_value="false"),
        DeclareLaunchArgument("live_ack", default_value=""),
        DeclareLaunchArgument("camera_source", default_value="ros"),
        DeclareLaunchArgument("lane_style", default_value="tape"),
        DeclareLaunchArgument("line_color", default_value="yellow"),
        DeclareLaunchArgument("controller_mode", default_value="pid"),
        DeclareLaunchArgument("sim_steering_limit_rad", default_value="0.35"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("tuning_config", default_value=config),
        DeclareLaunchArgument("mode_tuning_config", default_value=LaunchConfiguration("tuning_config")),
        DeclareLaunchArgument("foxglove_enabled", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8767"),
        DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
        OpaqueFunction(function=validate),
        Node(package="rcauto_cpp", executable="rcauto_vision", name="rcauto_cpp_vision",
             output="screen",
             parameters=[config, LaunchConfiguration("tuning_config"),
                         LaunchConfiguration("mode_tuning_config"), {
                 "vision.source": LaunchConfiguration("camera_source"),
                 "vision.lane_style": LaunchConfiguration("lane_style"),
                 "vision.line_color": LaunchConfiguration("line_color"),
                 "vision.hardware_mode": ParameterValue(LaunchConfiguration("hardware_enabled"),value_type=bool)}]),
        Node(package="rcauto_cpp", executable="rcauto_perception", name="rcauto_cpp_perception",
             output="screen", parameters=[config, {
                 "detection.hardware_mode": ParameterValue(LaunchConfiguration("hardware_enabled"),value_type=bool)}]),
        Node(package="rcauto_cpp", executable="rcauto_controller", name="rcauto_cpp_controller",
             output="screen", parameters=[config, LaunchConfiguration("tuning_config"),
                         LaunchConfiguration("mode_tuning_config"), {
                 "controller.mode": LaunchConfiguration("controller_mode"),
                 "controller.ai.model_path": LaunchConfiguration("model_path"),
                 "safety.hardware_mode": ParameterValue(LaunchConfiguration("hardware_enabled"), value_type=bool),
                 "safety.require_range": ParameterValue(LaunchConfiguration("require_range"), value_type=bool)}]),
        Node(package="rcauto_cpp", executable="rcauto_actuator", name="rcauto_cpp_actuator",
             output="screen", parameters=[config, {
                 "actuator.hardware_enabled": ParameterValue(
                     LaunchConfiguration("hardware_enabled"), value_type=bool),
                 "actuator.live_ack": LaunchConfiguration("live_ack"),
                 "actuator.max_steering_rad": ParameterValue(
                     LaunchConfiguration("sim_steering_limit_rad"), value_type=float)}]),
        Node(package="rcauto_cpp", executable="rcauto_simulator", name="rcauto_cpp_simulator",
             output="screen", condition=IfCondition(sim)),
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="rcauto_cpp_foxglove",
             output="screen", condition=IfCondition(LaunchConfiguration("foxglove_enabled")),
             parameters=[{"port": ParameterValue(LaunchConfiguration("foxglove_port"),
                                                 value_type=int),
                          "address": LaunchConfiguration("foxglove_address"),
                          "capabilities": ["clientPublish", "parameters",
                                           "parametersSubscribe", "services",
                                           "connectionGraph"],
                          "client_topic_whitelist": ["^/rcauto/operator/.*$"],
                          "param_whitelist": [r"^/rcauto_cpp_(controller|vision|perception|actuator)\..*$"],
                          "service_whitelist": ["^/rcauto_cpp_.*$"],
                          "topic_whitelist": ["^/rcauto/.*$", "^/parameter_events$"]}]),
    ])
