"""
Collision Recovery Behaviors for Australian Rover Challenge 2026

Purpose:
- Detect navigation failures (costmap obstacles, stuck conditions)
- Implement recovery strategies
- Track collision penalties
- Report failure modes to base station

Penalties:
- 10% per collision with supply cache or artificial obstacle during autonomous phase
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, BackUp, Spin
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan

import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Optional
import math
import time


class RecoveryState(Enum):
    """Recovery behavior state machine"""
    NORMAL = 0
    STUCK_DETECTED = 1
    BACKING_UP = 2
    SPINNING = 3
    CLEARING_COSTMAP = 4
    WAYPOINT_SKIP = 5
    FAILURE = 6


@dataclass
class CollisionEvent:
    """Record of collision/stuck event"""
    timestamp: float
    pose_x: float
    pose_y: float
    recovery_attempt: int
    success: bool
    recovery_type: str  # 'backup', 'spin', 'clear_costmap'


class CollisionRecoveryHandler(Node):
    """
    Detects stuck conditions and implements recovery behaviors
    
    Stuck detection criteria:
    1. Goal not progressing (odometry stuck for 10+ seconds)
    2. Costmap occupied at robot location
    3. Lidar/depth sensor detects obstacles
    4. Navigation node reports repeated failures
    
    Recovery sequence:
    1. Clear local costmap (removes noise)
    2. Back up 30 cm
    3. Spin 360�� to reacquire features
    4. Retry path planning
    5. If still stuck, skip waypoint
    """
    
    def __init__(self):
        super().__init__('collision_recovery')
        
        # Parameters
        self.declare_parameter('stuck_timeout', 10.0)  # seconds
        self.declare_parameter('min_forward_progress', 0.1)  # meters
        self.declare_parameter('max_recovery_attempts', 4)
        self.declare_parameter('enable_lidar_checking', False)  # If LIDAR available
        self.declare_parameter('obstacle_threshold_distance', 0.3)  # meters
        self.declare_parameter('autonomous_mode', True)
        
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.min_progress = self.get_parameter('min_forward_progress').value
        self.max_recovery = self.get_parameter('max_recovery_attempts').value
        self.enable_lidar = self.get_parameter('enable_lidar_checking').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold_distance').value
        self.autonomous_mode = self.get_parameter('autonomous_mode').value
        
        # State tracking
        self.state = RecoveryState.NORMAL
        self.collision_events: list[CollisionEvent] = []
        self.recovery_attempts = 0
        self.last_progress_pose = None
        self.last_progress_time = None
        self.navigation_goal = None
        
        # Current odometry
        self.current_pose = None
        self.current_odom = None
        self.obstacle_distance = float('inf')
        
        # Action clients
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.spin_client = ActionClient(self, Spin, 'spin')
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            reliable_qos
        )
        
        if self.enable_lidar:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                'scan',  # LaserScan topic if LIDAR available
                self.lidar_callback,
                reliable_qos
            )
        
        # Publishers
        self.recovery_status_pub = self.create_publisher(
            String,
            'collision_recovery/status',
            10
        )
        
        self.collision_penalty_pub = self.create_publisher(
            Float32,
            'collision_recovery/penalty_percent',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Timers
        self.create_timer(0.5, self.stuck_detection_loop)
        self.create_timer(2.0, self.status_update)
        
        self.get_logger().info('Collision Recovery Handler initialized')

    def odom_callback(self, msg: Odometry):
        """Update odometry and track movement"""
        self.current_pose = msg.pose.pose
        self.current_odom = msg
        
        # Initialize progress tracker
        if self.last_progress_pose is None:
            self.last_progress_pose = self.current_pose
            self.last_progress_time = self.get_clock().now()

    def lidar_callback(self, msg: LaserScan):
        """Update obstacle distance from LIDAR"""
        if len(msg.ranges) == 0:
            return
        
        # Filter out invalid readings
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            # Minimum distance to any obstacle
            self.obstacle_distance = min(valid_ranges)

    def stuck_detection_loop(self):
        """Main loop for stuck detection"""
        if self.state == RecoveryState.NORMAL:
            if self.detect_stuck_condition():
                self.state = RecoveryState.STUCK_DETECTED
                self.handle_stuck_condition()
        
        # Check progress towards goal
        self.check_progress_to_goal()

    def detect_stuck_condition(self) -> bool:
        """
        Detect if rover is stuck
        
        Criteria:
        1. No forward progress for stuck_timeout seconds
        2. LIDAR detects obstacles closer than threshold
        3. Multiple consecutive nav failures
        """
        if not self.current_pose or not self.last_progress_pose:
            return False
        
        # Check elapsed time
        elapsed = (self.get_clock().now() - self.last_progress_time).nanoseconds / 1e9
        
        if elapsed < self.stuck_timeout:
            return False
        
        # Check distance traveled
        dx = self.current_pose.position.x - self.last_progress_pose.position.x
        dy = self.current_pose.position.y - self.last_progress_pose.position.y
        distance_traveled = math.sqrt(dx**2 + dy**2)
        
        if distance_traveled < self.min_progress:
            self.get_logger().warning(
                f'Stuck detected: only {distance_traveled:.3f}m traveled in {elapsed:.1f}s'
            )
            return True
        
        # Check LIDAR (if available)
        if self.enable_lidar and self.obstacle_distance < self.obstacle_threshold:
            self.get_logger().warning(
                f'Obstacle at {self.obstacle_distance:.2f}m - stuck'
            )
            return True
        
        return False

    def handle_stuck_condition(self):
        """Execute recovery sequence"""
        self.recovery_attempts += 1
        
        if self.recovery_attempts > self.max_recovery:
            self.get_logger().error(
                f'Max recovery attempts ({self.max_recovery}) exceeded'
            )
            self.state = RecoveryState.FAILURE
            self.record_collision_event(success=False)
            return
        
        self.get_logger().info(
            f'Executing recovery behavior (attempt {self.recovery_attempts}/{self.max_recovery})'
        )
        
        # Recovery sequence
        self.execute_recovery_sequence()

    def execute_recovery_sequence(self):
        """Implement multi-step recovery"""
        if self.recovery_attempts == 1:
            # Step 1: Back up
            self.backup()
        elif self.recovery_attempts == 2:
            # Step 2: Spin to reacquire features
            self.spin()
        elif self.recovery_attempts == 3:
            # Step 3: Manual forward movement
            self.manual_forward()
        elif self.recovery_attempts >= 4:
            # Step 4: Give up and skip waypoint
            self.skip_waypoint()

    def backup(self):
        """Execute backup action"""
        self.state = RecoveryState.BACKING_UP
        
        if not self.backup_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('BackUp action server not available')
            self.execute_recovery_sequence()
            return
        
        goal = BackUp.Goal()
        goal.target = 0.3  # meters
        goal.max_speed = 0.1  # m/s
        
        self.backup_client.send_goal_async(goal).add_done_callback(
            self.recovery_result_callback
        )

    def spin(self):
        """Execute spin action"""
        self.state = RecoveryState.SPINNING
        
        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Spin action server not available')
            self.execute_recovery_sequence()
            return
        
        goal = Spin.Goal()
        goal.target_yaw = 2 * math.pi  # 360 degrees
        goal.max_speed = 0.5  # rad/s
        
        self.spin_client.send_goal_async(goal).add_done_callback(
            self.recovery_result_callback
        )

    def manual_forward(self):
        """Manual forward movement to break deadlock"""
        twist = Twist()
        twist.linear.x = 0.2  # m/s
        self.cmd_vel_pub.publish(twist)
        
        # Publish for 3 seconds
        self.create_timer(3.0, lambda: self.stop_movement())

    def stop_movement(self):
        """Stop rover movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.state = RecoveryState.NORMAL
        self.recovery_attempts = 0

    def skip_waypoint(self):
        """Skip current waypoint and continue to next"""
        self.state = RecoveryState.WAYPOINT_SKIP
        self.get_logger().warning('Skipping waypoint due to repeated failures')
        self.record_collision_event(success=False)
        
        # Signal to waypoint_handler to skip
        status_msg = String()
        status_msg.data = 'SKIP_WAYPOINT'
        self.recovery_status_pub.publish(status_msg)

    def recovery_result_callback(self, future):
        """Handle recovery action result"""
        try:
            result = future.result()
            if result.result:
                self.get_logger().info('Recovery action succeeded')
                self.record_collision_event(success=True)
                self.state = RecoveryState.NORMAL
                self.recovery_attempts = 0
                self.reset_progress_tracker()
            else:
                self.get_logger().warning('Recovery action failed, retrying')
                self.execute_recovery_sequence()
        except Exception as e:
            self.get_logger().error(f'Recovery action error: {e}')
            self.execute_recovery_sequence()

    def check_progress_to_goal(self):
        """Track progress towards navigation goal"""
        if not self.current_pose:
            return
        
        # If significant progress, reset tracker
        if self.last_progress_pose:
            dx = self.current_pose.position.x - self.last_progress_pose.position.x
            dy = self.current_pose.position.y - self.last_progress_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > self.min_progress:
                self.reset_progress_tracker()

    def reset_progress_tracker(self):
        """Reset progress tracking variables"""
        self.last_progress_pose = self.current_pose
        self.last_progress_time = self.get_clock().now()
        self.recovery_attempts = 0
        self.state = RecoveryState.NORMAL

    def record_collision_event(self, success: bool):
        """Record collision/stuck event"""
        if self.current_pose:
            event = CollisionEvent(
                timestamp=self.get_clock().now().nanoseconds / 1e9,
                pose_x=self.current_pose.position.x,
                pose_y=self.current_pose.position.y,
                recovery_attempt=self.recovery_attempts,
                success=success,
                recovery_type='backup' if self.recovery_attempts == 1 else 'spin'
            )
            self.collision_events.append(event)
        
        # Publish penalty (if autonomous mode)
        if self.autonomous_mode:
            penalty_percent = Float32(data=10.0)  # 10% per collision
            self.collision_penalty_pub.publish(penalty_percent)
            
            self.get_logger().info(
                f'Collision penalty: 10% (Total collisions: {len(self.collision_events)})'
            )

    def status_update(self):
        """Publish recovery status"""
        status_msg = String()
        import json
        status_data = {
            'state': self.state.name,
            'recovery_attempts': self.recovery_attempts,
            'total_collisions': len(self.collision_events),
            'autonomous_penalty_percent': 10.0 * len(self.collision_events),
            'recent_collisions': [
                {
                    'timestamp': evt.timestamp,
                    'pose': [evt.pose_x, evt.pose_y],
                    'recovery_type': evt.recovery_type,
                    'success': evt.success
                }
                for evt in self.collision_events[-5:]
            ]
        }
        status_msg.data = json.dumps(status_data)
        self.recovery_status_pub.publish(status_msg)

    def get_collision_report(self) -> dict:
        """Return collision statistics for final report"""
        total_collisions = len(self.collision_events)
        successful_recoveries = sum(1 for evt in self.collision_events if evt.success)
        
        return {
            'total_collisions': total_collisions,
            'successful_recoveries': successful_recoveries,
            'autonomous_penalty_percent': 10.0 * total_collisions,
            'events': [
                {
                    'timestamp': evt.timestamp,
                    'pose': [evt.pose_x, evt.pose_y],
                    'recovery_type': evt.recovery_type,
                    'success': evt.success,
                    'attempt': evt.recovery_attempt
                }
                for evt in self.collision_events
            ]
        }


def main(args=None):
    rclpy.init(args=args)
    recovery_handler = CollisionRecoveryHandler()
    rclpy.spin(recovery_handler)
    rclpy.shutdown()


if __name__ == '__main__':
    main()