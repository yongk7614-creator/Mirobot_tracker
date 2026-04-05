import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist  # 매카넘 휠 제어를 위한 Twist 추가
import math
from collections import deque

# [참고] 실물 로봇 연결 시 아래 주석을 해제하세요.
# from .wlkatapython import Mirobot

class MirobotTracker(Node):
    def __init__(self):
        super().__init__('mirobot_tracker')

        # [1] 데이터 수신 노드 (이벤트 기반)
        # 카메라가 마커를 인식할 때마다 최신 좌표를 '저장'만 합니다.
        self.subscription = self.create_subscription(
            PoseArray,
            'aruco_poses',
            self.listener_callback,
            10)

        # [2] 명령 전송 노드 (매카넘 휠 제어용)
        # 교수님 피드백대로 cmd_vel 토픽을 통해 하체를 제어합니다.
       # self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("--------------------------------------------------")
        self.get_logger().info("✅ ROS 2 이벤트 기반 협동 제어 시스템 가동")
        self.get_logger().info("🎯 목표 주차 간격: 250mm (허용범위: 235~265mm)")
        self.get_logger().info("--------------------------------------------------")

        # --- [물리 및 제어 파라미터] ---
        self.cam_x_offset = 80.0
        self.cam_pitch_deg = 15.0  
        self.optimal_dist = 250.0
        self.xy_tolerance = 15.0
        self.max_z = 415.0         
        self.min_z = 40.0          

        # --- [데이터 보관함] ---
        self.latest_raw_pose = None  # 가장 최근에 들어온 마커 좌표
        self.pose_history = deque(maxlen=5) # 필터용 큐

        # --- [3] 제어 타이머 (스케줄링) ---
        self.control_timer = self.create_timer(0.3, self.control_loop)

        # 실물 로봇 객체 (나중에 연결 시 주석 해제)
        # self.arm = Mirobot(portname='/dev/ttyUSB0')

    def listener_callback(self, msg):
        """
        데이터 수신 이벤트: 아루코 노드가 데이터를 보낼 때마다 실행됩니다.
        여기서는 연산을 하지 않고 '최신 데이터 갱신'만 담당하여 CPU 부하를 줄입니다.
        """
        if not msg.poses:
            self.latest_raw_pose = None
            return
        
        # 최신 좌표 저장 (이벤트 발생)
        self.latest_raw_pose = msg.poses[0]

    def control_loop(self):
        """
        제어 타이머 루프: ROS 2 스케줄러에 의해 0.3초마다 독립적으로 실행됩니다.
        가장 최근에 수신된 데이터를 바탕으로 로직을 판단합니다.
        """
        # 1. 최신 데이터가 없으면 즉시 종료 (아무것도 하지 않음)
        if self.latest_raw_pose is None:
            return

        # 2. 데이터 전처리 (단위 변환 및 필터링)
        raw_x = self.latest_raw_pose.position.x * 1000.0
        raw_y = self.latest_raw_pose.position.y * 1000.0
        raw_z = self.latest_raw_pose.position.z * 1000.0

        self.pose_history.append((raw_x, raw_y, raw_z))
        avg_x = sum(p[0] for p in self.pose_history) / len(self.pose_history)
        avg_y = sum(p[1] for p in self.pose_history) / len(self.pose_history)
        avg_z = sum(p[2] for p in self.pose_history) / len(self.pose_history)

        # 3. 좌표계 변환 (카메라 -> 베이스 동체)
        pitch_rad = math.radians(self.cam_pitch_deg)
        base_x = self.cam_x_offset + (avg_z * math.cos(pitch_rad)) - (avg_y * math.sin(pitch_rad))
        base_y = -avg_x
        base_z = (avg_z * math.sin(pitch_rad)) - (avg_y * math.cos(pitch_rad))
        xy_distance = math.sqrt(base_x**2 + base_y**2)

        # 4. 안전 한계 검사 (이상치 및 Z축)
        if xy_distance > 800.0 or base_z > 800.0: return
        if base_z < self.min_z or base_z > self.max_z:
           self.stop_wheels() # 위험 상황 시 바퀴 정지
           self.get_logger().warn(f"🚫 [Z축 한계] 범위를 벗어남 (Z: {base_z:.1f}mm)")
            return

        # 5. [핵심] 주차 및 도킹 판단 로직
        twist_msg = Twist()

        if xy_distance > (self.optimal_dist + self.xy_tolerance):
            # [전진] 목표보다 멀 때
            self.get_logger().warn(f"🛞 [전진] 간격: {xy_distance:.1f}mm")
            # twist_msg.linear.x = 0.1 # 0.1m/s 속도로 전진
            # self.cmd_vel_pub.publish(twist_msg)
            
        elif xy_distance < (self.optimal_dist - self.xy_tolerance):
            # [후진] 목표보다 가까울 때
            self.get_logger().warn(f"🛞 [후진] 간격: {xy_distance:.1f}mm")
            # twist_msg.linear.x = -0.1 # 0.1m/s 속도로 후진
            # self.cmd_vel_pub.publish(twist_msg)
            
        else:
            # [주차 완료 및 도킹] 명당(235~265mm) 진입 시
            self.stop_wheels() # 바퀴 멈춤
            self.get_logger().info(f"✅ [주차완료] 도킹 시작 (간격: {xy_distance:.1f}mm)")
            self.send_robot_command(base_x, base_y, base_z)

    def stop_wheels(self):
        """매카넘 휠에 정지 명령을 보냅니다."""
        # stop_msg = Twist()
        # self.cmd_vel_pub.publish(stop_msg)
        pass
    def send_robot_command(self, x, y, z):
        target_speed = 600
        self.get_logger().info(f"🤖 [ARM SEND] x:{x:.1f}, y:{y:.1f}, z:{z:.1f}, F:{target_speed}")
        # 실물 로봇 연결 시 아래 주석 해제
        # try:
        #     self.arm.set_p(x, y, z, 0.0, 0.0, 0.0, speed=target_speed)
        # except Exception as e:
        #     self.get_logger().error(f"로봇 통신 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MirobotTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_wheels() # 종료 시 안전을 위해 바퀴 정지
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
