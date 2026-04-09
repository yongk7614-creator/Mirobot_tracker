import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
from collections import deque

# from .wlkatapython import Mirobot


class MirobotTracker(Node):
    def __init__(self):
        super().__init__('mirobot_tracker')

        # 노드 구독 설정
        self.subscription = self.create_subscription(
            PoseArray, 'aruco_poses', self.listener_callback, 5
        )
        self.status_sub = self.create_subscription(
            String, 'arm_status', self.arm_status_callback, 5
        )
        self.wheel_status_sub = self.create_subscription(
            String, 'wheel_status', self.wheel_status_callback, 5
        )

        # Mirobot 스펙에 따른 offset 구성 및 동작 제한 범위
        self.cam_x_offset = 80.0
        self.cam_pitch_deg = 15.0
        self.max_z = 415.0
        self.min_z = 40.0

        # 상태 제어 변수
        self.pose_history = deque(maxlen=5)
        self.is_chassis_parked = False
        self.one_shot_timer = None

    def wheel_status_callback(self, msg):
        if msg.data == "STOPPED" and not self.is_chassis_parked:
            self.is_chassis_parked = True
            self.pose_history.clear()
            self.start_one_shot_timer()

    def arm_status_callback(self, msg):
        if msg.data == "DONE":
            self.start_one_shot_timer()

    def start_one_shot_timer(self):
        if self.one_shot_timer is not None:
            self.one_shot_timer.cancel()

        self.one_shot_timer = self.create_timer(0.2, self.execute_delayed_move)

    def execute_delayed_move(self):
        if self.one_shot_timer is not None:
            self.one_shot_timer.cancel()
        self.calculate_and_send_target()

    def listener_callback(self, msg):
        if not msg.poses:
            return

        raw_x = msg.poses[0].position.x * 1000.0
        raw_y = msg.poses[0].position.y * 1000.0
        raw_z = msg.poses[0].position.z * 1000.0
        self.pose_history.append((raw_x, raw_y, raw_z))

    def calculate_and_send_target(self):
        if len(self.pose_history) < 5:
            return

        avg_x = sum(p[0] for p in self.pose_history) / 5.0
        avg_y = sum(p[1] for p in self.pose_history) / 5.0
        avg_z = sum(p[2] for p in self.pose_history) / 5.0

        pitch_rad = math.radians(self.cam_pitch_deg)
        base_x = self.cam_x_offset + (avg_z * math.cos(pitch_rad)) - (avg_y * math.sin(pitch_rad))
        base_y = -avg_x
        base_z = (avg_z * math.sin(pitch_rad)) - (avg_y * math.cos(pitch_rad))

        horizontal_reach = math.sqrt(base_x ** 2 + base_y ** 2)
        limit_max_reach = 350.0
        limit_min_reach = 120.0

        is_safe = True
        if not (self.min_z <= base_z <= self.max_z):
            is_safe = False
        elif horizontal_reach > limit_max_reach:
            is_safe = False
        elif horizontal_reach < limit_min_reach:
            is_safe = False

        if is_safe:
            target_speed = 600
            self.get_logger().info(f"x:{base_x:.1f}, y:{base_y:.1f}, z:{base_z:.1f}")
            # self.arm.set_p(base_x, base_y, base_z, 0.0, 0.0, 0.0, speed=target_speed)


def main(args=None):
    rclpy.init(args=args)
    node = MirobotTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
