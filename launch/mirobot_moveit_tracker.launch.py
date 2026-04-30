from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("pose_topic", default_value="/aruco_pose_base"),
        DeclareLaunchArgument("wheel_status_topic", default_value="/wheel_status"),
        DeclareLaunchArgument("goal_topic", default_value="/mirobot_goal_pose"),
        DeclareLaunchArgument("sample_delay_sec", default_value="0.2"),
        DeclareLaunchArgument("sample_count", default_value="5"),
        DeclareLaunchArgument("offset_x", default_value="0.0"),
        DeclareLaunchArgument("offset_y", default_value="0.0"),
        DeclareLaunchArgument("offset_z", default_value="0.005"),
        DeclareLaunchArgument("goal_frame", default_value="base_link"),
        DeclareLaunchArgument("use_marker_orientation", default_value="true"),
        DeclareLaunchArgument("goal_qx", default_value="0.0"),
        DeclareLaunchArgument("goal_qy", default_value="0.0"),
        DeclareLaunchArgument("goal_qz", default_value="0.0"),
        DeclareLaunchArgument("goal_qw", default_value="1.0"),
        DeclareLaunchArgument("group_name", default_value="mirobot_group"),
        DeclareLaunchArgument("base_link_name", default_value="base_link"),
        DeclareLaunchArgument("end_effector_name", default_value="link6"),
        DeclareLaunchArgument("cartesian", default_value="false"),
        DeclareLaunchArgument("cartesian_max_step", default_value="0.0025"),
        DeclareLaunchArgument("cartesian_fraction_threshold", default_value="0.0"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("ignore_same_goal", default_value="true"),
    ]

    wheel_stop_parameters = {
        "pose_topic": LaunchConfiguration("pose_topic"),
        "wheel_status_topic": LaunchConfiguration("wheel_status_topic"),
        "goal_topic": LaunchConfiguration("goal_topic"),
        "sample_delay_sec": ParameterValue(
            LaunchConfiguration("sample_delay_sec"), value_type=float
        ),
        "sample_count": ParameterValue(
            LaunchConfiguration("sample_count"), value_type=int
        ),
        "offset_x": ParameterValue(LaunchConfiguration("offset_x"), value_type=float),
        "offset_y": ParameterValue(LaunchConfiguration("offset_y"), value_type=float),
        "offset_z": ParameterValue(LaunchConfiguration("offset_z"), value_type=float),
        "goal_frame": LaunchConfiguration("goal_frame"),
        "use_marker_orientation": ParameterValue(
            LaunchConfiguration("use_marker_orientation"), value_type=bool
        ),
        "goal_qx": ParameterValue(LaunchConfiguration("goal_qx"), value_type=float),
        "goal_qy": ParameterValue(LaunchConfiguration("goal_qy"), value_type=float),
        "goal_qz": ParameterValue(LaunchConfiguration("goal_qz"), value_type=float),
        "goal_qw": ParameterValue(LaunchConfiguration("goal_qw"), value_type=float),
    }

    moveit_goal_parameters = {
        "goal_pose_topic": LaunchConfiguration("goal_topic"),
        "group_name": LaunchConfiguration("group_name"),
        "base_link_name": LaunchConfiguration("base_link_name"),
        "end_effector_name": LaunchConfiguration("end_effector_name"),
        "cartesian": ParameterValue(
            LaunchConfiguration("cartesian"), value_type=bool
        ),
        "cartesian_max_step": ParameterValue(
            LaunchConfiguration("cartesian_max_step"), value_type=float
        ),
        "cartesian_fraction_threshold": ParameterValue(
            LaunchConfiguration("cartesian_fraction_threshold"), value_type=float
        ),
        "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
        "ignore_same_goal": ParameterValue(
            LaunchConfiguration("ignore_same_goal"), value_type=bool
        ),
    }

    return LaunchDescription(
        launch_args
        + [
            Node(
                package="mirobot_moveit_tracker",
                executable="wheel_stop_to_goal_node",
                name="wheel_stop_to_goal_node",
                output="screen",
                parameters=[wheel_stop_parameters],
            ),
            Node(
                package="mirobot_moveit_tracker",
                executable="moveit_goal_node",
                name="moveit_goal_node",
                output="screen",
                parameters=[moveit_goal_parameters],
            ),
        ]
    )
