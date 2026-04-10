import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
from collections import deque
from statistics import median

# 실물 미로봇 제어 시 주석 해제
# from .wlkatapython import Mirobot

class MirobotTracker(Node):
    def __init__(self):
        super().__init__('mirobot_tracker')

        self.declare_parameter('target_speed', 600)
        self.declare_parameter('max_range_x', 15.0)     
        self.declare_parameter('max_range_y', 15.0)     
        self.declare_parameter('max_range_z', 20.0)     
        self.declare_parameter('jump_limit_xy', 80.0)   
        self.declare_parameter('jump_limit_z', 100.0)   
        self.declare_parameter('max_reach', 350.0)      
        self.declare_parameter('min_reach', 120.0)      

        self.state_pub = self.create_publisher(String, 'tracker_state', 5)
        self.subscription = self.create_subscription(PoseArray, 'aruco_poses', self.listener_callback, 5)
        self.status_sub = self.create_subscription(String, 'arm_status', self.arm_status_callback, 5)
        self.wheel_status_sub = self.create_subscription(String, 'wheel_status', self.wheel_status_callback, 5)

        self.cam_x_offset = 80.0
        self.cam_pitch_deg = 15.0
        self.max_z = 415.0
        self.min_z = 40.0

        self.pose_history = deque(maxlen=5)
        self.is_chassis_parked = False
        self.one_shot_timer = None
        self.waiting_for_frames = False
        self.is_complete = False
        self.is_failed = False
        self.last_target = None

        self.waiting_start_time = None
        self.frame_wait_timeout_sec = 1.0
        self.frame_retry_count = 0
        self.max_frame_retry = 5
        self.iteration_count = 0
        self.max_iterations = 10

        self.publish_state("IDLE")

    def publish_state(self, state_str): 
        msg = String()
        msg.data = state_str
        self.state_pub.publish(msg)
        
    def _reset_state(self):
        self.waiting_for_frames = False
        self.waiting_start_time = None
        self.pose_history.clear()
        self._destroy_one_shot_timer()

    def wheel_status_callback(self, msg):
        if self.is_complete or self.is_failed:
            return

        if msg.data == "STOPPED":
            if self.is_chassis_parked:
                return
            self.is_chassis_parked = True
            self.publish_state("WAITING_FOR_FRAMES")
            self.start_settling_timer()

        elif msg.data == "MOVING":
            self.is_chassis_parked = False
            self._reset_state()
            self.publish_state("MOVING")

    def arm_status_callback(self, msg):
        if self.is_complete or self.is_failed:
            return

        if msg.data == "DONE":
            if self.iteration_count >= self.max_iterations:
                self.is_failed = True
                self.publish_state("FAILED_MAX_ITER")
                return

            self.publish_state("WAITING_FOR_FRAMES")
            self.start_settling_timer()

    def start_settling_timer(self):
        self._destroy_one_shot_timer()
        self.one_shot_timer = self.create_timer(0.2, self.on_settling_timer_expired)

    def on_settling_timer_expired(self):
        self._destroy_one_shot_timer()
        self.pose_history.clear()
        self.waiting_for_frames = True
        self.waiting_start_time = self.get_clock().now()
        self.frame_retry_count = 0  

    def _destroy_one_shot_timer(self):
        if self.one_shot_timer is not None:
            self.one_shot_timer.cancel()
            self.destroy_timer(self.one_shot_timer)
            self.one_shot_timer = None

    def is_pose_valid(self, x, y, z):
        if not (-1000.0 <= x <= 1000.0) or not (-1000.0 <= y <= 1000.0) or not (50.0 <= z <= 2000.0):
            return False

        if len(self.pose_history) > 0:
            prev_x, prev_y, prev_z = self.pose_history[-1]
            dx, dy, dz = abs(x - prev_x), abs(y - prev_y), abs(z - prev_z)

            jump_limit_xy = self.get_parameter('jump_limit_xy').value
            jump_limit_z = self.get_parameter('jump_limit_z').value

            if dx > jump_limit_xy or dy > jump_limit_xy or dz > jump_limit_z:
                self.get_logger().warn(f"Outlier rejected (dx:{dx:.1f}, dy:{dy:.1f}, dz:{dz:.1f})", throttle_duration_sec=1.0)
                return False
        return True

    def is_pose_stable(self):
        if len(self.pose_history) < 5:
            return False

        xs = [p[0] for p in self.pose_history]
        ys = [p[1] for p in self.pose_history]
        zs = [p[2] for p in self.pose_history]

        rx = max(xs) - min(xs)
        ry = max(ys) - min(ys)
        rz = max(zs) - min(zs)
        
        if rx > self.get_parameter('max_range_x').value or \
           ry > self.get_parameter('max_range_y').value or \
           rz > self.get_parameter('max_range_z').value:
            return False
        return True

    def listener_callback(self, msg):
        if self.is_complete or self.is_failed:
            return

        if not msg.poses:
            self._check_frame_wait_timeout()
            return

        closest_pose = min(msg.poses, key=lambda p: p.position.z)
        raw_x = closest_pose.position.x * 1000.0
        raw_y = closest_pose.position.y * 1000.0
        raw_z = closest_pose.position.z * 1000.0

        if not self.is_pose_valid(raw_x, raw_y, raw_z):
            self._check_frame_wait_timeout()
            return

        self.pose_history.append((raw_x, raw_y, raw_z))

        if self.waiting_for_frames and len(self.pose_history) == 5:
            if not self.is_pose_stable():
                self.waiting_start_time = self.get_clock().now() 
                return

            self.waiting_for_frames = False
            self.waiting_start_time = None
            self.calculate_and_send_target()
        else:
            self._check_frame_wait_timeout()

    def _check_frame_wait_timeout(self):
        if not self.waiting_for_frames or self.waiting_start_time is None:
            return

        elapsed = (self.get_clock().now() - self.waiting_start_time).nanoseconds / 1e9
        if elapsed > self.frame_wait_timeout_sec:
            self.frame_retry_count += 1
            if self.frame_retry_count >= self.max_frame_retry:
                self._reset_state()
                self.is_failed = True
                self.publish_state("FAILED_TIMEOUT")
                return

            self.pose_history.clear()
            self.waiting_start_time = self.get_clock().now()

    def is_safe(self, x, y, z):
        horizontal_reach = math.sqrt(x**2 + y**2)
        if not (self.min_z <= z <= self.max_z) or \
           horizontal_reach > self.get_parameter('max_reach').value or \
           horizontal_reach < self.get_parameter('min_reach').value:
            return False
        return True

    def calculate_and_send_target(self):
        if len(self.pose_history) < 5:
            return

        filt_x = median([p[0] for p in self.pose_history])
        filt_y = median([p[1] for p in self.pose_history])
        filt_z = median([p[2] for p in self.pose_history])

        pitch_rad = math.radians(self.cam_pitch_deg)
        base_x = self.cam_x_offset + (filt_z * math.cos(pitch_rad)) - (filt_y * math.sin(pitch_rad))
        base_y = -filt_x
        base_z = (filt_z * math.sin(pitch_rad)) - (filt_y * math.cos(pitch_rad))

        if self.last_target is not None:
            cmd_dx = base_x - self.last_target[0]
            cmd_dy = base_y - self.last_target[1]
            cmd_dz = base_z - self.last_target[2]
            command_delta = math.sqrt(cmd_dx**2 + cmd_dy**2 + cmd_dz**2)
            residual_3d = math.sqrt(base_y**2 + (base_z - self.min_z)**2) 
            
            if command_delta <= 1.0:
                self.is_complete = True
                self.publish_state("COMPLETE_CONVERGED")
                return
            
            if residual_3d <= 3.0: 
                self.is_complete = True
                self.publish_state("COMPLETE_ON_TARGET")
                return

        if self.is_safe(base_x, base_y, base_z):
            self.iteration_count += 1
            self.publish_state("ALIGNING")
            self.last_target = (base_x, base_y, base_z)
        
        self.arm.set_p(base_x, base_y, base_z, 0.0, 0.0, 0.0, speed=self.get_parameter('target_speed').value)
        else:
            self.publish_state("WAITING_FOR_FRAMES")

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
