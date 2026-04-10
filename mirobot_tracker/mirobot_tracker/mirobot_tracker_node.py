import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
from collections import deque

from .wlkatapython import Mirobot

class MirobotTracker(Node):
    def __init__(self):
        super().__init__('mirobot_tracker')

        # 노드 구독 설정
        # 30fps로 들어오는 아루코 마커 좌표 배열 수신
        self.subscription = self.create_subscription(
            PoseArray, 'aruco_poses', self.listener_callback, 5)
        # 로봇 팔이 동작을 마쳤을 때 보내는 신호(DONE) 대기
        self.status_sub = self.create_subscription(
            String, 'arm_status', self.arm_status_callback, 5)
        # 매카넘 휠이 정차했다는 신호(STOPPED) 수신
        self.wheel_status_sub = self.create_subscription(
            String, 'wheel_status', self.wheel_status_callback, 5)

        # Mirobot 스펙에 따른 offset 구성 및 동작 제한 범위
        self.cam_x_offset = 80.0 # 로봇 베이스 중심과 8cm 떨어짐
        self.cam_pitch_deg = 15.0  # 위로 15도 기울어짐
        self.max_z = 415.0         # 로봇 팔이 도달 가능한 최대 높이
        self.min_z = 40.0          # 바닥 충돌 방지 최소 높이

        # 상태 제어 변수
        self.pose_history = deque(maxlen=5)  # 5개의 평균 벡터 추출 위함
        self.is_chassis_parked = False      # 매카넘 휠 정차 완료 확인 여부
        self.one_shot_timer = None          # 0.2초 대기를 위한 일회성 타이머

    def wheel_status_callback(self, msg):
        if msg.data == "STOPPED" and not self.is_chassis_parked:
            self.is_chassis_parked = True
            self.pose_history.clear() # 흔들린 과거 데이터 삭제
            self.start_one_shot_timer() # 0.2초 타이머 시작

    def arm_status_callback(self, msg):
        if msg.data == "DONE":
            self.start_one_shot_timer() # 0.2초 타이머 시작

    def start_one_shot_timer(self):
        if self.one_shot_timer is not None:
            self.one_shot_timer.cancel()
        
        self.one_shot_timer = self.create_timer(0.2, self.execute_delayed_move)

    def execute_delayed_move(self):
        self.one_shot_timer.cancel()
        self.calculate_and_send_target()

    def listener_callback(self, msg):
        if not msg.poses:
            return # 마커 안 보이면 return
        
        raw_x = msg.poses[0].position.x * 1000.0
        raw_y = msg.poses[0].position.y * 1000.0
        raw_z = msg.poses[0].position.z * 1000.0
        self.pose_history.append((raw_x, raw_y, raw_z))

    def calculate_and_send_target(self):
        if len(self.pose_history) < 5: 
            return

        # 5개 데이터 평균 계산
        avg_x = sum(p[0] for p in self.pose_history) / 5
        avg_y = sum(p[1] for p in self.pose_history) / 5
        avg_z = sum(p[2] for p in self.pose_history) / 5

        # 좌표계 변환 (카메라 -> Mirobot 베이스)
        pitch_rad = math.radians(self.cam_pitch_deg)
        base_x = self.cam_x_offset + (avg_z * math.cos(pitch_rad)) - (avg_y * math.sin(pitch_rad))
        base_y = -avg_x
        base_z = (avg_z * math.sin(pitch_rad)) - (avg_y * math.cos(pitch_rad))

        # Mirobot 한계 도달 스펙
        horizontal_reach = math.sqrt(base_x**2 + base_y**2)
        LIMIT_MAX_REACH = 350.0  # 최대 뻗을 수 있는 거리 (mm)
        LIMIT_MIN_REACH = 120.0  # 최소 안전 거리 (mm) - 베이스 충돌 방지
        
        # 안전 범위 검사 및 명령 전송
        is_safe = True
        if not (self.min_z <= base_z <= self.max_z):
            is_safe = False
        elif horizontal_reach > LIMIT_MAX_REACH:
            is_safe = False
        elif horizontal_reach < LIMIT_MIN_REACH:
            is_safe = False
            
        if is_safe:
            target_speed = 600
            self.get_logger().info(f"x:{base_x:.1f}, y:{base_y:.1f}, z:{base_z:.1f}")
            self.arm.set_p(base_x, base_y, base_z, 0.0, 0.0, 0.0, speed=target_speed)

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
