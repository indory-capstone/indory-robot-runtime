from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("indoory_xlerobot_bridge"), "config", "nav2_params.yaml"]
                ),
            ),
            Node(package="nav2_controller", executable="controller_server", output="screen", parameters=[params_file]),
            Node(package="nav2_planner", executable="planner_server", output="screen", parameters=[params_file]),
            Node(package="nav2_behaviors", executable="behavior_server", output="screen", parameters=[params_file]),
            Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen", parameters=[params_file]),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[params_file],
                remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{"autostart": True}, {"node_names": lifecycle_nodes}, params_file],
            ),
        ]
    )
