import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mirobot_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mirobot_moveit_tracker"),
                "launch",
                "mirobot_moveit_tracker.launch.py",
            )
        ),
        launch_arguments={
            "pose_topic": "/aruco_pose_base",
            "wheel_status_topic": "/wheel_status",
            "goal_topic": "/mirobot_goal_pose",
            "sample_delay_sec": "0.2",
            "sample_count": "5",
            "offset_x": "0.0",
            "offset_y": "0.0",
            "offset_z": "0.005",
            "goal_frame": "base_link",
            "use_marker_orientation": "true",
            "goal_qx": "0.0",
            "goal_qy": "0.0",
            "goal_qz": "0.0",
            "goal_qw": "1.0",
            "group_name": "mirobot_group",
            "base_link_name": "base_link",
            "end_effector_name": "link6",
            "cartesian": "false",
            "cartesian_max_step": "0.0025",
            "cartesian_fraction_threshold": "0.0",
            "execute": "true",
            "ignore_same_goal": "true",
        }.items(),
    )

    wlkata_moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wlkata_mirobot_moveit_config"),
                "launch",
                "demo.launch.py",
            )
        )
    )

    wlkata_arm_move_node = Node(
        package="wlkata_arm_move",
        executable="mirobot_moveit_move",
        name="mirobot_moveit_move",
        output="screen",
    )

    return LaunchDescription(
        [
            wlkata_moveit_demo_launch,
            mirobot_tracker_launch,
            wlkata_arm_move_node,
        ]
    )
