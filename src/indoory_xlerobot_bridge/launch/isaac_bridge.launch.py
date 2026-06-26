from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("pub_port", default_value="5555"),
            DeclareLaunchArgument("pull_port", default_value="5556"),
            DeclareLaunchArgument("rep_port", default_value="5557"),
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("command_schema", default_value="auto"),
            DeclareLaunchArgument("command_frame", default_value="body"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/xlerobot/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/xlerobot/odom"),
            DeclareLaunchArgument("scan_topic", default_value="/xlerobot/scan"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            Node(
                package="indoory_xlerobot_bridge",
                executable="isaac_sim_zmq_bridge",
                name="xlerobot_isaac_sim_zmq_bridge",
                output="screen",
                parameters=[
                    {
                        "sim_host": LaunchConfiguration("sim_host"),
                        "pub_port": ParameterValue(LaunchConfiguration("pub_port"), value_type=int),
                        "pull_port": ParameterValue(
                            LaunchConfiguration("pull_port"), value_type=int
                        ),
                        "rep_port": ParameterValue(LaunchConfiguration("rep_port"), value_type=int),
                        "robot_id": ParameterValue(LaunchConfiguration("robot_id"), value_type=int),
                        "command_schema": LaunchConfiguration("command_schema"),
                        "command_frame": LaunchConfiguration("command_frame"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_tf"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
