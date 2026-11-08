"""Official F1TENTH Gym 2D physics with RCauto's C++ controller and KSTI map."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("rcauto_cpp"))
    gym_share = Path(get_package_share_directory("f1tenth_gym_ros"))
    track = share / "sim" / "maps" / "ksti2026_f1tenth"
    map_yaml = str(track) + ".yaml"
    centerline = str(share / "sim" / "maps" / "ksti2026_centerline.csv")
    config = str(share / "config" / "robot.yaml")
    foxglove = LaunchConfiguration("foxglove_enabled")
    rviz = LaunchConfiguration("rviz_enabled")

    return LaunchDescription([
        DeclareLaunchArgument("foxglove_enabled", default_value="true"),
        DeclareLaunchArgument("rviz_enabled", default_value="false"),
        DeclareLaunchArgument("controller_mode", default_value="pid"),
        DeclareLaunchArgument("model_path", default_value=""),
        Node(
            package="f1tenth_gym_ros",
            executable="gym_bridge",
            name="bridge",
            output="screen",
            parameters=[{
                "map_path": str(track),
                "map_img_ext": ".png",
                "num_agent": 1,
                "kb_teleop": False,
                "sx": 2.22044,
                "sy": -0.01520,
                "stheta": -1.561007,
                "use_sim_time": False,
            }],
            remappings=[("/drive", "/ego_racecar/drive")],
        ),
        Node(
            package="rcauto_cpp",
            executable="rcauto_f1tenth_adapter",
            name="rcauto_f1tenth_adapter",
            output="screen",
            parameters=[{"centerline_path": centerline,
                         "lookahead_m": 0.24,
                         "lane_max_offset_m": 0.23,
                         "command_timeout_s": 0.35}],
        ),
        Node(
            package="rcauto_cpp",
            executable="rcauto_controller",
            name="rcauto_cpp_controller",
            output="screen",
            parameters=[config, {
                "controller.mode": LaunchConfiguration("controller_mode"),
                "controller.ai.model_path": LaunchConfiguration("model_path"),
                "safety.hardware_mode": False,
                "safety.require_range": True,
                "safety.range_stop_m": 0.18,
            }],
        ),
        Node(
            package="rcauto_cpp",
            executable="rcauto_actuator",
            name="rcauto_cpp_actuator",
            output="screen",
            parameters=[config, {
                "actuator.hardware_enabled": False,
                "actuator.live_ack": "",
            }],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="rcauto_f1tenth_map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_yaml,
                "use_sim_time": False,
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="rcauto_f1tenth_lifecycle",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["rcauto_f1tenth_map_server"],
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="rcauto_f1tenth_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": ParameterValue(
                Command([FindExecutable(name="xacro"), " ",
                         str(gym_share / "launch" / "ego_racecar.xacro")]),
                value_type=str)}],
            remappings=[("/robot_description", "/ego_robot_description")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rcauto_f1tenth_rviz",
            arguments=["-d", str(gym_share / "launch" / "gym_bridge.rviz")],
            condition=IfCondition(rviz),
            output="screen",
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="rcauto_cpp_foxglove",
            output="screen",
            condition=IfCondition(foxglove),
            parameters=[{
                "port": 8767,
                "address": "127.0.0.1",
                "capabilities": ["clientPublish", "parameters",
                                 "parametersSubscribe", "services",
                                 "connectionGraph"],
                "client_topic_whitelist": ["^/rcauto/operator/.*$"],
                "param_whitelist": [r"^/rcauto_cpp_controller\..*$"],
                "service_whitelist": ["^/rcauto_cpp_.*$"],
                "topic_whitelist": ["^/rcauto/.*$", "^/ego_racecar/.*$",
                                    "^/map$", "^/parameter_events$"],
            }],
        ),
    ])
