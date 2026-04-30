import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("mirobot_moveit_tracker")
    default_params = os.path.join(package_share, "config", "tracker_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Parameter file for tracker and MoveIt goal nodes.",
            ),
            Node(
                package="mirobot_moveit_tracker",
                executable="wheel_stop_to_goal_node",
                name="wheel_stop_to_goal_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="mirobot_moveit_tracker",
                executable="moveit_goal_node",
                name="moveit_goal_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
