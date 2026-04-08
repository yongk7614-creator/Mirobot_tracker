import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
from collections import deque

# 실물 미로봇 제어 시 주석 해제
# from .wlkatapython import Mirobot

class MirobotTracker(Node):
    def __init__(self):
        super().__init__('mirobot_tracker')

        # 카메라 30Hz 비전 데이터 수신
        self.subscription = self.create_subscription(
            PoseArray, 'aruco_poses', self.listener_callback, 5)
        # 로봇 팔 동작 완료 신호 수신
        self.status_sub = self.create_subscription(
            String, 'arm_status', self.arm_status_callback, 5)
        # 매카넘 휠 정차 완료 신호 수신
        self.wheel_status_sub = self.create_subscription(
            String, 'wheel_status', self.wheel_status_callback, 5)

        # 하드웨어 및 설치 환경 파라미터
        self.cam_x_offset = 80.0     # 로봇 베이스 중심과 카메라 간의 거리 보정
        self.cam_pitch_deg = 15.0    # 카메라 상향 기울기
        self.max_z = 415.0           # 팔 상승 최대 한계
        self.min_z = 40.0            # 바닥 충돌 방지 

        # 제어 상태 변수
        self.pose_history = deque(maxlen=5)  # 노이즈 필터링용 5프레임 이동 평균 버퍼
        self.is_chassis_parked = False       # 하체 정차 여부 플래그
        self.one_shot_timer = None           # 0.2초 물리적 안정화 대기용 타이머
        
        self.waiting_for_frames = False      # 버퍼 초기화 후 새 5프레임 수집 대기 상태
        self.last_target = None              # 직전 타겟 좌표 (도킹 완료시 오차 계산용)

    def wheel_status_callback(self, msg):
         if msg.data == "STOPPED" and not self.is_chassis_parked:
            self.is_chassis_parked = True
            self.start_settling_timer()

    def arm_status_callback(self, msg):
        if msg.data == "DONE":
            self.start_settling_timer()

    def start_settling_timer(self):
        if self.one_shot_timer is not None:
            self.one_shot_timer.cancel()
        
        self.one_shot_timer = self.create_timer(0.2, self.on_settling_timer_expired)

    def on_settling_timer_expired(self):
        self.one_shot_timer.cancel()
        self.pose_history.clear() 
        self.waiting_for_frames = True

    def listener_callback(self, msg):
        if not msg.poses:
            return 
        
        # 미터(m) 단위를 밀리미터(mm)로 변환
        raw_x = msg.poses[0].position.x * 1000.0
        raw_y = msg.poses[0].position.y * 1000.0
        raw_z = msg.poses[0].position.z * 1000.0
        self.pose_history.append((raw_x, raw_y, raw_z))

        # 버퍼가 비워진 후, 정지 상태의 깨끗한 데이터가 5개 꽉 차면 계산 시작
        if self.waiting_for_frames and len(self.pose_history) == 5:
            self.waiting_for_frames = False
            self.calculate_and_send_target()

    def is_safe(self, x, y, z):
        horizontal_reach = math.sqrt(x**2 + y**2)
        LIMIT_MAX_REACH = 350.0  # 최대 도달 가능 거리
        LIMIT_MIN_REACH = 120.0  # 자기 몸체 충돌 방지 최소 거리
        
        if not (self.min_z <= z <= self.max_z):
             return False
        if horizontal_reach > LIMIT_MAX_REACH:
            return False
        if horizontal_reach < LIMIT_MIN_REACH:
            return False
        
        return True

    def calculate_and_send_target(self):
        if len(self.pose_history) < 5: 
            return

        # 5프레임 이동 평균 산출
        avg_x = sum(p[0] for p in self.pose_history) / 5
        avg_y = sum(p[1] for p in self.pose_history) / 5
        avg_z = sum(p[2] for p in self.pose_history) / 5

        # 카메라 -> 로봇 베이스 기준 좌표계 변환
        pitch_rad = math.radians(self.cam_pitch_deg)
        base_x = self.cam_x_offset + (avg_z * math.cos(pitch_rad)) - (avg_y * math.sin(pitch_rad))
        base_y = -avg_x
        base_z = (avg_z * math.sin(pitch_rad)) - (avg_y * math.cos(pitch_rad))

        # 제어 종료 조건: 이전 목표와의 3차원 유클리디안 거리 오차가 2mm 이하인지 확인
        if self.last_target is not None:
            dx = base_x - self.last_target[0]
            dy = base_y - self.last_target[1]
            dz = base_z - self.last_target[2]
            error_dist = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # 오차가 허용 범위 내라면 G-code 전송 중단하고 도킹 프로세스로 전환
            if error_dist <= 2.0:
                return 

        # 물리적 도달 한계 안전성 검사 통과 시 명령 전송
        if self.is_safe(base_x, base_y, base_z):
            target_speed = 600
            self.last_target = (base_x, base_y, base_z) # 다음 스텝 오차 비교를 위해 현재 타겟 저장
            
            # 실물 로봇 연결 시 주석 해제하여 G-code 전송
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
