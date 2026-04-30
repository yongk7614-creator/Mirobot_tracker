import copy
import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymoveit2 import MoveIt2


class MoveItGoalNode(Node):
    def __init__(self):
        super().__init__("moveit_goal_node")

        self.declare_parameter("goal_pose_topic", "/mirobot_goal_pose")
        self.declare_parameter("group_name", "mirobot_group")
        self.declare_parameter("base_link_name", "base_link")
        self.declare_parameter("end_effector_name", "link6")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("cartesian", False)
        self.declare_parameter("cartesian_max_step", 0.0025)
        self.declare_parameter("cartesian_fraction_threshold", 0.0)
        self.declare_parameter("execute", True)
        self.declare_parameter("ignore_same_goal", True)

        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.base_link_name = str(self.get_parameter("base_link_name").value)
        self.end_effector_name = str(self.get_parameter("end_effector_name").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.cartesian = bool(self.get_parameter("cartesian").value)
        self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value)
        self.cartesian_fraction_threshold = float(
            self.get_parameter("cartesian_fraction_threshold").value
        )
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.ignore_same_goal = bool(self.get_parameter("ignore_same_goal").value)

        self._last_goal = None  # type: Optional[PoseStamped]
        self._busy = False
        self._lock = threading.Lock()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.base_link_name,
            end_effector_name=self.end_effector_name,
            group_name=self.group_name,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self.goal_pose_callback,
            10,
        )

    def goal_pose_callback(self, msg):
        self.get_logger().info(
            "Received MoveIt goal request: frame=%s x=%.4f y=%.4f z=%.4f"
            % (
                msg.header.frame_id,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )
        )
        
        with self._lock:
            if self._busy:
                self.get_logger().warn("MoveIt is busy. Ignoring new goal.")
                return

            if self.ignore_same_goal and self._is_same_goal(msg, self._last_goal):
                self.get_logger().info("Same goal received. Ignoring.")
                return

            self._busy = True

        worker = threading.Thread(
            target=self._process_goal,
            args=(copy.deepcopy(msg),),
            daemon=True,
        )
        worker.start()

    def _process_goal(self, goal_pose):
        try:
            self.send_goal_to_moveit(goal_pose)
            self._last_goal = copy.deepcopy(goal_pose)
        except Exception as exc:
            self.get_logger().error("Failed to send goal to MoveIt: %s" % str(exc))
        finally:
            with self._lock:
                self._busy = False

    def send_goal_to_moveit(self, goal_pose):
        if not goal_pose.header.frame_id:
            raise ValueError("Goal pose has empty frame_id.")

        if goal_pose.header.frame_id != self.base_link_name:
            raise ValueError(
                "Goal pose frame_id '%s' does not match expected base frame '%s'."
                % (goal_pose.header.frame_id, self.base_link_name)
            )

        q = goal_pose.pose.orientation
        if self._is_zero_quaternion(q.x, q.y, q.z, q.w):
            raise ValueError("Goal pose has invalid zero quaternion.")

        qx, qy, qz, qw = self._normalize_quaternion(q.x, q.y, q.z, q.w)

        position = [
            float(goal_pose.pose.position.x),
            float(goal_pose.pose.position.y),
            float(goal_pose.pose.position.z),
        ]
        quat_xyzw = [qx, qy, qz, qw]

        self.get_logger().info(
            "Goal validated. Ready to send to MoveIt: frame=%s x=%.4f y=%.4f z=%.4f"
            % (
                goal_pose.header.frame_id,
                position[0],
                position[1],
                position[2],
            )
        )

        if self.cartesian:
            self.get_logger().info("Sending cartesian goal to MoveIt.")
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
        else:
            self.get_logger().info("Sending joint-space goal to MoveIt.")
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=False,
            )
            
        if self.execute_motion:
            self.moveit2.wait_until_executed()
            self.get_logger().info("MoveIt goal excution completed.")
        else:
            self.get_logger().info("MoveIt goal was submitted without excution wait.")
            

    def _is_same_goal(self, a, b):
        if b is None:
            return False

        pa = a.pose.position
        pb = b.pose.position
        qa = a.pose.orientation
        qb = b.pose.orientation

        return (
            a.header.frame_id == b.header.frame_id
            and abs(pa.x - pb.x) < 1e-6
            and abs(pa.y - pb.y) < 1e-6
            and abs(pa.z - pb.z) < 1e-6
            and abs(qa.x - qb.x) < 1e-6
            and abs(qa.y - qb.y) < 1e-6
            and abs(qa.z - qb.z) < 1e-6
            and abs(qa.w - qb.w) < 1e-6
        )

    @staticmethod
    def _is_zero_quaternion(x, y, z, w):
        return abs(x) < 1e-12 and abs(y) < 1e-12 and abs(z) < 1e-12 and abs(w) < 1e-12

    @staticmethod
    def _normalize_quaternion(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            raise ValueError("Quaternion norm is too small.")
        return x / norm, y / norm, z / norm, w / norm


def main(args=None):
    rclpy.init(args=args)
    node = MoveItGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
