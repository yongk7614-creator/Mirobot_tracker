import copy

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class WheelStopToGoalNode(Node):
    def __init__(self):
        super().__init__("wheel_stop_to_goal_node")

        defaults = {
            "pose_topic": "/aruco_poses",
            "wheel_status_topic": "/wheel_status",
            "goal_topic": "/mirobot_goal_pose",
            "sample_delay_sec": 0.2,
            "sample_count": 5,
            "offset_x": 0.0,
            "offset_y": 0.0,
            "offset_z": 0.0,
            "goal_frame": "base_link",
            "use_marker_orientation": True,
            "goal_qx": 0.0,
            "goal_qy": 0.0,
            "goal_qz": 0.0,
            "goal_qw": 1.0,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        self.latest_pose = None
        self.prev_is_stopped = False
        self.collecting = False
        self.sample_buffer = []
        self.delay_timer = None

        self.pose_sub = self.create_subscription(
            PoseArray, self.pose_topic, self.pose_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, self.wheel_status_topic, self.status_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

    def reset_sampling(self):
        self.collecting = False
        self.sample_buffer = []
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None

    def pose_callback(self, msg):
        if len(msg.poses) == 0:
            return

        pose_msg = PoseStamped()
        pose_msg.header = copy.deepcopy(msg.header)
        pose_msg.pose = copy.deepcopy(msg.poses[0])
        self.latest_pose = pose_msg

        if not self.collecting:
            return

        self.sample_buffer.append(copy.deepcopy(pose_msg))
        if len(self.sample_buffer) >= self.sample_count:
            self.publish_averaged_goal()
            self.collecting = False
            self.sample_buffer = []

    def status_callback(self, msg):
        is_stopped = msg.data.strip().lower() == "stopped"

        if not is_stopped:
            self.prev_is_stopped = False
            self.reset_sampling()
            return

        if self.prev_is_stopped:
            return

        if self.latest_pose is None:
            self.get_logger().warn("No ArUco pose received yet.")
            return

        self.prev_is_stopped = True
        self.reset_sampling()
        self.delay_timer = self.create_timer(self.sample_delay_sec, self.start_sampling_once)

        self.get_logger().info(
            "Wheel stopped. Waiting %.3f sec before collecting %d samples."
            % (self.sample_delay_sec, self.sample_count)
        )

    def start_sampling_once(self):
        self.reset_sampling()
        self.collecting = True
        self.get_logger().info("Started ArUco pose sampling.")

    def publish_averaged_goal(self):
        if not self.sample_buffer:
            self.get_logger().warn("No samples collected.")
            return

        avg_x = sum(p.pose.position.x for p in self.sample_buffer) / len(self.sample_buffer)
        avg_y = sum(p.pose.position.y for p in self.sample_buffer) / len(self.sample_buffer)
        avg_z = sum(p.pose.position.z for p in self.sample_buffer) / len(self.sample_buffer)

        goal_pose = copy.deepcopy(self.sample_buffer[-1])
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.goal_frame
        goal_pose.pose.position.x = avg_x + self.offset_x
        goal_pose.pose.position.y = avg_y + self.offset_y
        goal_pose.pose.position.z = avg_z + self.offset_z

        if not self.use_marker_orientation:
            goal_pose.pose.orientation.x = self.goal_qx
            goal_pose.pose.orientation.y = self.goal_qy
            goal_pose.pose.orientation.z = self.goal_qz
            goal_pose.pose.orientation.w = self.goal_qw

        self.goal_pub.publish(goal_pose)
        self.get_logger().info(
            "Averaged pose published: x=%.4f y=%.4f z=%.4f"
            % (
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelStopToGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
