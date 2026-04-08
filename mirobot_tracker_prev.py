import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
from collections import deque
from statistics import median

# 실물 미로봇 제어 시 주석 해제 (미로봇 전용 파이썬 API 라이브러리)
# from .wlkatapython import Mirobot

class MirobotTracker(Node):
    def __init__(self):
        super().__init__('mirobot_tracker')

     
        # ROS 2 파라미터 선언 
        # 로봇 팔이 움직일 속도 (mm/min)
        self.declare_parameter('target_speed', 600)
        # 5개의 프레임이 모였을 때, X축 데이터가 이 값(15mm) 이상 흔들리면 불안정한 것으로 간주
        self.declare_parameter('max_range_x', 15.0)     
        # Y축 데이터 흔들림 허용치 (15mm)
        self.declare_parameter('max_range_y', 15.0)     
        # Z축(깊이) 데이터 흔들림 허용치 (20mm)
        self.declare_parameter('max_range_z', 20.0)     
        # 직전 프레임과 비교해 X, Y 좌표가 한 번에 80mm 이상 튀면 노이즈(오류)로 간주하고 버림
        self.declare_parameter('jump_limit_xy', 80.0)   
        # Z축(깊이) 방향으로 한 번에 100mm 이상 튀면 버림
        self.declare_parameter('jump_limit_z', 100.0)   
        # 로봇 팔이 최대로 뻗을 수 있는 물리적 한계 반경 (350mm)
        self.declare_parameter('max_reach', 350.0)      
        # 로봇 팔이 자기 몸통(베이스)과 부딪히지 않기 위한 최소 접근 금지 반경 (120mm)
        self.declare_parameter('min_reach', 120.0)      

        # 통신 설정 (퍼블리셔 및 서브스크라이버)
        # 현재 이 노드(추적기)의 상태(대기중, 정렬중, 완료 등)를 외부로 알리는 퍼블리셔 (큐 사이즈 5)
        self.state_pub = self.create_publisher(String, 'tracker_state', 5)
        
        # 카메라 노드로부터 아루코 마커의 3D 좌표 배열을 초당 30번(30Hz)씩 받아오는 서브스크라이버
        self.subscription = self.create_subscription(PoseArray, 'aruco_poses', self.listener_callback, 5)
        # 로봇 팔이 이동을 끝냈을 때 'DONE' 신호를 받는 서브스크라이버
        self.status_sub = self.create_subscription(String, 'arm_status', self.arm_status_callback, 5)
        # 매카넘 휠(하체)이 멈췄을 때 'STOPPED', 움직일 때 'MOVING' 신호를 받는 서브스크라이버
        self.wheel_status_sub = self.create_subscription(String, 'wheel_status', self.wheel_status_callback, 5)

        # 물리적 설치 환경 파라미터 (단위: mm, deg)
        # 카메라가 로봇 팔의 중심축(베이스)으로부터 X축 방향으로 80mm 앞에 설치되어 있음
        self.cam_x_offset = 80.0
        # 카메라가 수평을 보지 않고, 상향으로 15도 들려서 설치되어 있음
        self.cam_pitch_deg = 15.0
        # 로봇 팔이 위로 올라갈 수 있는 최대 높이 제한 (415mm)
        self.max_z = 415.0
        # 로봇 팔이 바닥에 닿거나 충돌하지 않도록 하는 최소 높이 제한 (40mm)
        self.min_z = 40.0

        # 제어 상태 관리 변수
        # 최신 5개의 마커 좌표만 저장하는 슬라이딩 윈도우 큐
        self.pose_history = deque(maxlen=5)
        # 현재 하체가 주차 상태인지 기억하는 플래그
        self.is_chassis_parked = False
        # 0.2초 안정화 대기를 수행할 ROS 타이머 객체를 담을 변수
        self.one_shot_timer = None
        # 현재 큐에 새로운 프레임 5개가 다 찰 때까지 기다리고 있는 상태인지 나타내는 플래그
        self.waiting_for_frames = False
        
        # 전체 도킹 프로세스가 성공적으로 끝났음을 알리는 플래그
        self.is_complete = False
        # 에러(타임아웃, 횟수 초과 등)로 인해 도킹이 실패하여 멈췄음을 알리는 플래그
        self.is_failed = False
        # 오차 비교를 위해 직전에 로봇에게 명령을 내렸던 타겟 좌표(X,Y,Z)를 저장
        self.last_target = None

        # 예외 처리 (타임아웃 및 재시도 제한) 변수
        # 프레임 수집을 시작한 시간을 기록 (타임아웃 체크용)
        self.waiting_start_time = None
        # 마커가 안 보이거나 데이터가 불안정할 때 기다려주는 최대 시간 (1.0초)
        self.frame_wait_timeout_sec = 1.0
        # 타임아웃 발생 시 다시 시도한 횟수를 카운트
        self.frame_retry_count = 0
        # 최대 5번 연속으로 타임아웃이 발생하면 완전히 실패로 간주하고 시스템 정지
        self.max_frame_retry = 5

        # 로봇 팔이 오차를 줄이기 위해 미세 조정을 시도한 횟수 카운트
        self.iteration_count = 0
        # 최대 10번 움직여도 도킹 조건이 안 맞으면 그만두고 실패 처리
        self.max_iterations = 10

        # 노드가 시작되었음을 터미널에 알림
        self.get_logger().info("MirobotTracker: 강건 제어 및 상태 머신 노드 가동 완료.")
        # 초기 상태를 'IDLE(대기)'로 퍼블리시
        self.publish_state("IDLE")

    
    def publish_state(self, state_str): 
        # 현재 노드의 상태 문자열을 토픽으로 발행하고, 터미널에도 출력하는 통합 함수
        msg = String()
        msg.data = state_str
        self.state_pub.publish(msg) # 'tracker_state' 토픽으로 문자열 전송
        
    def _reset_state(self):
        # 진행 중이던 대기 상태, 큐, 타이머를 모두 백지화(리셋)하는 함수. 하체가 다시 움직일 때 사용
        self.waiting_for_frames = False
        self.waiting_start_time = None
        self.pose_history.clear()      # 큐 안에 있던 데이터 전부 삭제
        self._destroy_one_shot_timer() # 돌고 있던 타이머 강제 종료

    def wheel_status_callback(self, msg):
        # 하체(매카넘 휠) 노드에서 보내는 상태 메시지를 처리하는 콜백 함수
        # 이미 도킹이 끝났거나 실패했다면 하체의 움직임은 무시함
        if self.is_complete or self.is_failed:
            return

        # 하체가 목적지에 도착해서 멈췄다는 신호
        if msg.data == "STOPPED":
            # 이미 정차 처리되어 있다면 중복 실행 방지
            if self.is_chassis_parked:
                return
            self.is_chassis_parked = True
            # 상태를 프레임 대기 중으로 변경
            self.publish_state("WAITING_FOR_FRAMES")
            # 0.2초 물리적 진동 안정화 타이머 가동
            self.start_settling_timer()

        # 하체가 다시 움직이기 시작했다는 신호
        elif msg.data == "MOVING":
            self.is_chassis_parked = False
            # 움직이는 동안 찍힌 데이터는 다 쓰레기이므로 리셋 함수 호출
            self._reset_state()
            self.publish_state("MOVING")

    def arm_status_callback(self, msg):
        # 로봇 팔(Mirobot) 제어기에서 명령 수행을 완료했다고 보내는 메시지 처리
        if self.is_complete or self.is_failed:
            return

        # 로봇 팔이 1스텝 동작을 끝마쳤다는 신호
        if msg.data == "DONE":
            # 무한히 꼼지락거리는 것을 막기 위해, 최대 제어 횟수(10번)를 넘었는지 검사
            if self.iteration_count >= self.max_iterations:
                self.is_failed = True # 실패 플래그 ON
                self.publish_state("FAILED_MAX_ITER")
                return

            self.publish_state("WAITING_FOR_FRAMES")
            # 팔이 멈출 때 발생한 진동을 가라앉히기 위해 0.2초 타이머 시작
            self.start_settling_timer()

    def start_settling_timer(self):
        # 0.2초짜리 일회성 ROS 타이머를 생성하는 함수
        # 혹시 이전에 돌고 있던 타이머가 있다면 안전하게 파괴
        self._destroy_one_shot_timer()
        # 0.2초(200ms) 뒤에 self.on_settling_timer_expired 함수를 딱 한 번 실행하도록 설정
        self.one_shot_timer = self.create_timer(0.2, self.on_settling_timer_expired)

    def on_settling_timer_expired(self):
        # 0.2초 타이머가 끝나면(진동이 다 가라앉으면) ROS가 자동으로 호출하는 함수
        # 타이머 메모리 해제
        self._destroy_one_shot_timer()
        # 진동이 가라앉는 0.2초 동안 큐에 들어와 있던 흔들리는 데이터를 전부 삭제
        self.pose_history.clear()
        
        # 이제부터 들어오는 깨끗한 프레임 5개를 수집하겠다는 플래그 ON
        self.waiting_for_frames = True
        # 타임아웃 계산을 위해 현재 시간을 기록
        self.waiting_start_time = self.get_clock().now()
        # 새롭게 수집을 시작하므로, 재시도 횟수는 0으로 리셋
        self.frame_retry_count = 0  

    def _destroy_one_shot_timer(self):
        # ROS 타이머 객체를 안전하게 종료하고 메모리에서 지우는 유틸리티 함수
        if self.one_shot_timer is not None:
            self.one_shot_timer.cancel()
            self.destroy_timer(self.one_shot_timer)
            self.one_shot_timer = None

    def is_pose_valid(self, x, y, z):
        # 카메라에서 들어온 좌표가 물리적으로 말이 되는 값인지 1차 필터링하는 함수
        # 절대 범위 검사: 상식적으로 카메라가 볼 수 있는 1미터~2미터 밖의 데이터면 버림
        if not (-1000.0 <= x <= 1000.0) or not (-1000.0 <= y <= 1000.0) or not (50.0 <= z <= 2000.0):
            return False

        # Jump 검사: 직전 프레임 대비 값이 순간이동(순간적으로 튀었는지)했는지 검사
        if len(self.pose_history) > 0:
            # 큐의 맨 마지막(가장 최근) 데이터 꺼내기
            prev_x, prev_y, prev_z = self.pose_history[-1]
            # 현재 좌표와 직전 좌표의 차이(절댓값) 계산
            dx, dy, dz = abs(x - prev_x), abs(y - prev_y), abs(z - prev_z)

            # ROS 파라미터에서 현재 설정된 허용치(limit) 값을 읽어옴
            jump_limit_xy = self.get_parameter('jump_limit_xy').value
            jump_limit_z = self.get_parameter('jump_limit_z').value

            # 허용치를 넘어서 크게 튀었다면 노이즈(빛 반사 등)로 간주하고 버림
            if dx > jump_limit_xy or dy > jump_limit_xy or dz > jump_limit_z:
                # 경고 메시지로 터미널이 도배되는 것을 막기 위해 1초에 한 번만 출력되도록 throttle 적용
                self.get_logger().warn(f"이상점 감지 거부됨 (dx:{dx:.1f}, dy:{dy:.1f}, dz:{dz:.1f})", throttle_duration_sec=1.0)
                return False
                
        # 모든 검사를 통과한 깨끗한 데이터임
        return True

    def is_pose_stable(self):
        # 큐에 모인 5개의 데이터가 얼마나 흔들리고 있는지 안정성을 평가하는 함수
        # 5개가 다 안 모였으면 검사 불가
        if len(self.pose_history) < 5:
            return False

        # 큐에 있는 5개 데이터에서 x, y, z 리스트를 각각 추출
        xs = [p[0] for p in self.pose_history]
        ys = [p[1] for p in self.pose_history]
        zs = [p[2] for p in self.pose_history]

        # ROS 파라미터에서 각 축별 최대 허용 흔들림(Range) 값을 읽어옴
        MAX_RANGE_X = self.get_parameter('max_range_x').value
        MAX_RANGE_Y = self.get_parameter('max_range_y').value
        MAX_RANGE_Z = self.get_parameter('max_range_z').value

        # (최댓값 - 최솟값)을 통해 5개 프레임 동안 데이터가 얼마나 요동쳤는지(Range) 계산
        rx, ry, rz = max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs)
        
        # 하나라도 허용치 이상으로 흔들렸다면 아직 안정화되지 않은 것으로 간주 (False 반환)
        if rx > MAX_RANGE_X or ry > MAX_RANGE_Y or rz > MAX_RANGE_Z:
            self.get_logger().info(f"안정성 미달. 프레임 갱신 대기 (rx:{rx:.1f}, ry:{ry:.1f}, rz:{rz:.1f})", throttle_duration_sec=1.0)
            return False
            
        # 5개의 데이터가 오밀조밀하게 모여있는 훌륭한 상태임
        return True

    def listener_callback(self, msg):
        # 카메라 노드에서 30Hz 속도로 쏴주는 아루코 마커 좌표 메시지를 처리하는 메인 콜백 함수
        # 도킹이 이미 종료되었으면 더 이상 카메라 데이터를 처리하지 않고 무시함
        if self.is_complete or self.is_failed:
            return

        # 카메라 화면에 마커가 아무것도 안 잡혔을 때
        if not msg.poses:
            # 혹시 기다리던 중이었다면 타임아웃 시간이 지났는지 체크
            self._check_frame_wait_timeout()
            return

        # 여러 개의 마커가 인식되었을 경우, 카메라와 가장 가까운(Z값이 가장 작은) 마커 1개만 타겟으로 선정
        closest_pose = min(msg.poses, key=lambda p: p.position.z)

        # 미터(m) 단위를 밀리미터(mm) 단위로 변환하여 저장
        raw_x = closest_pose.position.x * 1000.0
        raw_y = closest_pose.position.y * 1000.0
        raw_z = closest_pose.position.z * 1000.0

        # 데이터가 너무 튀지 않았는지 1차 검사 (실패하면 타임아웃만 체크하고 종료)
        if not self.is_pose_valid(raw_x, raw_y, raw_z):
            self._check_frame_wait_timeout()
            return

        # 1차 검사를 통과한 데이터를 큐의 맨 끝에 추가 (6개째면 제일 오래된 1번째 데이터가 자동 삭제됨)
        self.pose_history.append((raw_x, raw_y, raw_z))

        # 현재 5프레임을 모으고 있는 상태이고, 큐에 5개가 꽉 찼다면
        if self.waiting_for_frames and len(self.pose_history) == 5:
            # 5개가 모였어도 진동이 심한지(stable) 2차 검사. 만약 불안정하면
            if not self.is_pose_stable():
                # 큐를 비우지 않음, 다음 0.033초 뒤에 들어올 새 데이터가 과거 데이터를 밀어내도록 냅둠 (슬라이딩 윈도우 유지)
                # 대신, 무한히 안 모이는 것을 막기 위해 타임아웃 계산용 시작 시간만 갱신해줌
                self.waiting_start_time = self.get_clock().now() 
                return

            # 안정성 검사까지 통과 후 타겟 계산을 시작함
            self.waiting_for_frames = False
            self.waiting_start_time = None
            self.calculate_and_send_target()
        else:
            # 아직 5개가 다 안 찼다면 타임아웃이 지났는지 체크
            self._check_frame_wait_timeout()

    def _check_frame_wait_timeout(self):
        # 마커가 가려지거나 노이즈로 인해 프레임 수집이 멈춰버리는 무한 대기(Deadlock)를 방지하는 함수
        # 프레임 수집 모드가 아니거나, 시간이 기록되지 않았으면 검사 생략
        if not self.waiting_for_frames or self.waiting_start_time is None:
            return

        # 대기 시작 후 지금까지 몇 초가 흘렀는지 계산 (나노초 -> 초 변환)
        elapsed = (self.get_clock().now() - self.waiting_start_time).nanoseconds / 1e9
        
        # 설정한 타임아웃(1.0초)이 지났다면
        if elapsed > self.frame_wait_timeout_sec:
            self.frame_retry_count += 1 # 실패 카운트 증가
            
            # 재시도 횟수가 한계(5회, 총 5초)를 넘으면
            if self.frame_retry_count >= self.max_frame_retry:
                # 제어를 완전히 포기하고 상태를 실패로 변경
                self._reset_state()
                self.is_failed = True
                self.publish_state("FAILED_TIMEOUT")
                return

            # 아직 재시도 기회가 남았다면, 큐에 이상한 값이 껴서 막힌 것일 수 있으니 큐를 싹 비우고 리트라이
            self.pose_history.clear()
            self.waiting_start_time = self.get_clock().now() # 시간 다시 0초부터 측정

    def is_safe(self, x, y, z):
        # 계산된 목표 좌표가 로봇 팔이 뻗을 수 있는 한계를 넘거나, 자기 몸체를 찌르지 않는지 검사하는 안전망
        # 로봇 베이스 중심(원점)에서 목표 지점까지의 2D 평면상 직선거리(도달 반경) 계산 (피타고라스 정리)
        horizontal_reach = math.sqrt(x**2 + y**2)
        
        # ROS 파라미터에서 안전 반경 값 읽어오기
        MAX_REACH = self.get_parameter('max_reach').value
        MIN_REACH = self.get_parameter('min_reach').value

        # 목표 Z(높이)가 범위를 벗어났거나, 너무 멀거나, 몸통에 너무 가까우면 동작 거부
        if not (self.min_z <= z <= self.max_z) or horizontal_reach > MAX_REACH or horizontal_reach < MIN_REACH:
            return False
            
        return True

    def calculate_and_send_target(self):
        # 최종적으로 로봇이 이동할 좌표를 계산하고, 완료 여부를 판별한 뒤 G-code를 전송하는 메인 함수
        if len(self.pose_history) < 5:
            return

        # 5개 데이터 중 튀는 값을 배제하기 위해 리스트를 정렬한 후 한가운데 있는 Median을 추출
        filt_x = median([p[0] for p in self.pose_history])
        filt_y = median([p[1] for p in self.pose_history])
        filt_z = median([p[2] for p in self.pose_history])

        # 좌표 변환: 카메라가 기울어진 15도(Pitch) 각도를 삼각함수를 이용해 풀어서, 로봇 베이스 기준의 올바른 X, Y, Z로 변환
        pitch_rad = math.radians(self.cam_pitch_deg)
        base_x = self.cam_x_offset + (filt_z * math.cos(pitch_rad)) - (filt_y * math.sin(pitch_rad))
        base_y = -filt_x
        base_z = (filt_z * math.sin(pitch_rad)) - (filt_y * math.cos(pitch_rad))

        # 종료 판별 로직: 직전 스텝에서 명령을 내렸던 타겟이 존재한다면 비교 시작
        if self.last_target is not None:
            # 명령값 변화량 계산 (수렴 정체 감지용)
            cmd_dx = base_x - self.last_target[0]
            cmd_dy = base_y - self.last_target[1]
            cmd_dz = base_z - self.last_target[2]
            # 직전에 로봇에게 가라고 한 위치와, 지금 가라고 할 위치의 3차원 거리 차이
            command_delta = math.sqrt(cmd_dx**2 + cmd_dy**2 + cmd_dz**2)

            # 3D 잔차(목표 오차) 산출
            # Y축(좌우)이 0에 맞고, Z축(높이)이 목표한 최하단(min_z)에 도달했는지를 거리로 계산
            residual_3d = math.sqrt(base_y**2 + (base_z - self.min_z)**2) 
            
            # 종료 조건 1: 명령 변화량이 1.0mm 이하라는 건 로봇이 목표에 도달했거나 한계치에서 제자리걸음 중이라는 뜻
            if command_delta <= 1.0:
                self.is_complete = True
                self.publish_state("COMPLETE_CONVERGED")
                return
            
            # 종료 조건 2: 목표로 한 Y, Z 위치와의 3차원 오차가 3.0mm 이내로 들어왔다면 완벽한 성공
            if residual_3d <= 3.0: 
                self.is_complete = True
                self.publish_state("COMPLETE_ON_TARGET")
                return

        # 4. 안전망 검사: 계산된 좌표가 로봇을 부수지 않는 안전한 위치라면
        if self.is_safe(base_x, base_y, base_z):
            # ROS 파라미터에서 현재 설정된 이동 속도를 가져옴
            target_speed = self.get_parameter('target_speed').value
            
            # 제어(이동) 시도 횟수 1 증가
            self.iteration_count += 1
            self.publish_state("ALIGNING")
            self.get_logger().info(f"🚀 [SEND] x={base_x:.1f}, y={base_y:.1f}, z={base_z:.1f} (Step: {self.iteration_count})")
            
            # 다음 번 동작 시 비교를 위해 이번에 내린 명령 좌표를 저장해둠
            self.last_target = (base_x, base_y, base_z)

            # 실물 로봇에 연결 시 아래 주석을 풀면 Python 파라미터가 텍스트 형태의 G-code로 변환되어 시리얼로 전송됨
            # self.arm.set_p(base_x, base_y, base_z, 0.0, 0.0, 0.0, speed=target_speed)
        else:
            # 안전 구역 밖이라 명령을 포기했다면, 다시 프레임 수집 대기 상태로 전환
            self.publish_state("WAITING_FOR_FRAMES")

def main(args=None):
    """이 스크립트가 실행될 때 가장 먼저 호출되는 ROS 2 메인 함수"""
    rclpy.init(args=args)           # ROS 2 시스템 초기화
    node = MirobotTracker()         # 우리가 만든 MirobotTracker 노드 객체 생성
    try:
        rclpy.spin(node)            # 노드가 종료되지 않고 콜백 함수들을 무한히 처리하도록 루프 실행
    except KeyboardInterrupt:
        pass                        # 사용자가 터미널에서 Ctrl+C를 누르면 에러 없이 자연스럽게 루프 종료
    finally:
        node.destroy_node()         # 노드가 사용하던 메모리와 통신 포트 정리
        rclpy.shutdown()            # ROS 2 시스템 완전 종료

if __name__ == '__main__':
    main()
