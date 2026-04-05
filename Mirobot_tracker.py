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

        self.subscription = self.create_subscription(
            PoseArray, 'aruco_poses', self.listener_callback, 10)
        self.status_sub = self.create_subscription(
            String, 'arm_status', self.arm_status_callback, 10)
        self.wheel_status_sub = self.create_subscription(
            String, 'wheel_status', self.wheel_status_callback, 10)

        # Mirobot 스펙에 따른 offset 구성 및 동작 제한 범위
        self.cam_x_offset = 80.0
        self.cam_pitch_deg = 15.0  
        self.max_z = 415.0         
        self.min_z = 40.0          

        # 상태 제어 변수
        self.pose_history = deque(maxlen=5)  # 5개의 평균 벡터 추출 위함
        self.is_first_move = True           
        self.is_chassis_parked = False      # 매카넘 휠 정차 확인 플래그

        self.get_logger().info("Mirobot Tracker 노드가 가동되었습니다.")

    def wheel_status_callback(self, msg):
        if msg.data == "STOPPED" and not self.is_chassis_parked:
            self.is_chassis_parked = True
            self.pose_history.clear()

    def listener_callback(self, msg):
        if not msg.poses:
            return
        
        raw_x = msg.poses[0].position.x * 1000.0
        raw_y = msg.poses[0].position.y * 1000.0
        raw_z = msg.poses[0].position.z * 1000.0
        self.pose_history.append((raw_x, raw_y, raw_z))

        # 차체 정차 후 정지 상태의 데이터가 5개 꽉 차면 최초 명령 전송
        if self.is_first_move and self.is_chassis_parked and len(self.pose_history) == 5:
            self.is_first_move = False  
            self.calculate_and_send_target()

    def arm_status_callback(self, msg):
        if msg.data == "DONE":
            self.calculate_and_send_target()

    def calculate_and_send_target(self):
        if len(self.pose_history) < 5: 
            return

        # 5개 데이터 평균 계산
        avg_x = sum(p[0] for p in self.pose_history) / 5
        avg_y = sum(p[1] for p in self.pose_history) / 5
        avg_z = sum(p[2] for p in self.pose_history) / 5

        # 좌표계 변환 (카메라 15도 pitch 고려)
        pitch_rad = math.radians(self.cam_pitch_deg)
        base_x = self.cam_x_offset + (avg_z * math.cos(pitch_rad)) - (avg_y * math.sin(pitch_rad))
        base_y = -avg_x
        base_z = (avg_z * math.sin(pitch_rad)) - (avg_y * math.cos(pitch_rad))

        # 안전 범위 검사 및 명령 전송
        if self.min_z <= base_z <= self.max_z:
            target_speed = 600
            self.get_logger().info(f"명령 하달 -> x:{base_x:.1f}, y:{base_y:.1f}, z:{base_z:.1f}")
            # self.arm.set_p(base_x, base_y, base_z, 0.0, 0.0, 0.0, speed=target_speed)
        else:
            self.get_logger().warn(f"Z축 안전 범위를 벗어났습니다 (Z: {base_z:.1f})")

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
