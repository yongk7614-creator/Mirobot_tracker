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

        # [1] ROS 2 통신 설정 (구독자)
        # 카메라 30Hz 비전 데이터 수신
        self.subscription = self.create_subscription(
            PoseArray, 'aruco_poses', self.listener_callback, 5)
        # 로봇 팔 동작 완료 신호 수신
        self.status_sub = self.create_subscription(
            String, 'arm_status', self.arm_status_callback, 5)
        # 매카넘 휠 정차 완료 신호 수신
        self.wheel_status_sub = self.create_subscription(
            String, 'wheel_status', self.wheel_status_callback, 5)

        # [2] 하드웨어 및 설치 환경 파라미터 (mm, deg)
        self.cam_x_offset = 80.0     # 로봇 베이스 중심과 카메라 간의 거리 보정
        self.cam_pitch_deg = 15.0    # 카메라 하향 기울기
        self.max_z = 415.0           # 팔 상승 최대 한계
        self.min_z = 40.0            # 바닥 충돌 방지 하한계

        # [3] 제어 상태 변수
        self.pose_history = deque(maxlen=5)  # 노이즈 필터링용 5프레임 이동 평균 버퍼
        self.is_chassis_parked = False       # 하체 정차 여부 플래그
        self.one_shot_timer = None           # 0.2초 물리적 안정화 대기용 타이머
        
        self.waiting_for_frames = False      # 버퍼 초기화 후 새 5프레임 수집 대기 상태
        self.last_target = None              # 직전 타겟 좌표 (도킹 완료 오차 계산용)

    def wheel_status_callback(self, msg):
        """매카넘 휠이 목적지에 정차했을 때 최초 1회 실행"""
        if msg.data == "STOPPED" and not self.is_chassis_parked:
            self.is_chassis_parked = True
            self.start_settling_timer()

    def arm_status_callback(self, msg):
        """로봇 팔이 한 스텝 동작을 끝냈을 때 실행 (피드백 제어 루프)"""
        if msg.data == "DONE":
            self.start_settling_timer()

    def start_settling_timer(self):
        """기구적 잔진동을 가라앉히기 위해 0.2초 논블로킹 대기 시작"""
        if self.one_shot_timer is not None:
            self.one_shot_timer.cancel()
        
        self.one_shot_timer = self.create_timer(0.2, self.on_settling_timer_expired)

    def on_settling_timer_expired(self):
        """0.2초 안정화가 끝나면 흔들리던 버퍼를 비우고 정밀 수집 모드로 전환"""
        self.one_shot_timer.cancel()
        
        self.pose_history.clear() 
        self.waiting_for_frames = True

    def listener_callback(self, msg):
        """30Hz로 들어오는 비전 데이터를 수신하여 버퍼에 저장"""
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
        """로봇 팔의 작업 반경(Workspace) 및 충돌 구역 침범 여부 검사"""
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
        """데이터 필터링, 좌표계 변환, 오차 검사 후 최종 제어 명령 하달"""
        if len(self.pose_history) < 5: 
            return

        # 1. 5프레임 이동 평균 산출
        avg_x = sum(p[0] for p in self.pose_history) / 5
