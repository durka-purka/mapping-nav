"""
Navigation Manager for Australian Rover Challenge
Handles autonomous navigation with landmark detection and recovery
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformListener, Buffer
import numpy as np
from enum import Enum
import logging

class NavigationState(Enum):
    IDLE = 0
    NAVIGATING = 1
    EXPLORING = 2
    RECOVERING = 3
    COMPLETED = 4


class NavigationManager(Node):
    """
    Main navigation orchestrator for autonomous rover
    
    Responsibilities:
    - Coordinate Nav2 for landmark navigation
    - Handle frontier exploration
    - Manage collision recovery
    - Track mission state
    """
    
    def __init__(self):
        super().__init__('navigation_manager')
        
        # Setup logging
        self.logger = logging.getLogger('NavigationManager')
        
        # Parameters
        self.declare_parameter('nav_timeout', 300)  # 5 minutes per waypoint
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('landmark_list_file', '')
        self.declare_parameter('exploration_enabled', True)
        
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.max_retries = self.get_parameter('max_retries').value
        self.exploration_enabled = self.get_parameter('exploration_enabled').value
        
        # State tracking
        self.state = NavigationState.IDLE
        self.current_waypoint_idx = 0
        self.waypoints = []
        self.retry_count = 0
        self.start_pose = None
        
        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        
        # Subscriptions with custom QoS for reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            qos_profile
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timers
        self.create_timer(0.1, self.update_loop)
        self.create_timer(1.0, self.status_update)
        
        self.get_logger().info('Navigation Manager initialized')

    def initialize_mission(self, landmark_poses: list) -> bool:
        """
        Initialize navigation mission with landmark waypoints
        
        Args:
            landmark_poses: List of PoseStamped objects for landmarks
            
        Returns:
            True if initialization successful
        """
        try:
            # Wait for Nav2 to be available
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('Nav2 action server not available')
                return False
            
            self.waypoints = landmark_poses
            self.current_waypoint_idx = 0
            self.state = NavigationState.NAVIGATING
            
            self.get_logger().info(
                f'Mission initialized with {len(landmark_poses)} landmarks'
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f'Mission initialization failed: {e}')
            return False

    def navigate_to_landmark(self, landmark_idx: int) -> bool:
        """
        Send goal to Nav2 to navigate to specific landmark
        
        Args:
            landmark_idx: Index in waypoints list
            
        Returns:
            True if goal accepted
        """
        if landmark_idx >= len(self.waypoints):
            self.get_logger().error(f'Invalid landmark index: {landmark_idx}')
            return False
        
        goal = NavigateToPose.Goal()
        goal.pose = self.waypoints[landmark_idx]
        
        self.get_logger().info(
            f'Navigating to landmark {landmark_idx}: '
            f'({goal.pose.pose.position.x:.2f}, '
            f'{goal.pose.pose.position.y:.2f})'
        )
        
        # Send goal asynchronously
        self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_callback
        ).add_done_callback(self.nav_goal_response_callback)
        
        self.retry_count = 0
        return True

    def nav_feedback_callback(self, msg):
        """Handle navigation feedback from Nav2"""
        remaining_distance = msg.feedback.navigation_feedback.remaining_distance
        
        self.get_logger().debug(
            f'Navigation progress: {remaining_distance:.2f}m remaining'
        )

    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance/rejection"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warning(
                'Navigation goal rejected, initiating recovery'
            )
            self.initiate_recovery()
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Wait for result asynchronously
        self.goal_handle.get_result_async().add_done_callback(
            self.nav_result_callback
        )

    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        
        if result.result:  # Success
            self.get_logger().info(
                f'Successfully reached landmark '
                f'{self.current_waypoint_idx}'
            )
            self.current_waypoint_idx += 1
            
            # Continue to next landmark
            if self.current_waypoint_idx < len(self.waypoints):
                self.navigate_to_landmark(self.current_waypoint_idx)
            else:
                # All landmarks visited, start exploration
                self.state = NavigationState.EXPLORING
                self.get_logger().info('All landmarks visited, beginning exploration')
        else:
            self.get_logger().warning('Navigation failed, attempting recovery')
            self.initiate_recovery()

    def initiate_recovery(self):
        """Handle navigation failure with recovery behavior"""
        self.state = NavigationState.RECOVERING
        self.retry_count += 1
        
        if self.retry_count >= self.max_retries:
            self.get_logger().error(
                f'Max retries ({self.max_retries}) exceeded for landmark '
                f'{self.current_waypoint_idx}'
            )
            # Skip to next landmark
            self.current_waypoint_idx += 1
            self.retry_count = 0
            
            if self.current_waypoint_idx < len(self.waypoints):
                self.navigate_to_landmark(self.current_waypoint_idx)
            return
        
        # Attempt recovery: simple backoff + retry
        self.get_logger().info(
            f'Recovery attempt {self.retry_count}/{self.max_retries}'
        )
        
        # Back up slightly
        twist_msg = Twist()
        twist_msg.linear.x = -0.2
        self.cmd_vel_pub.publish(twist_msg)
        
        # Schedule retry after short delay
        self.create_timer(2.0, lambda: self.navigate_to_landmark(
            self.current_waypoint_idx
        ), callback_group=None)

    def odom_callback(self, msg: Odometry):
        """Update internal state with odometry"""
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
        
        self.current_pose = msg.pose.pose

    def update_loop(self):
        """Main update loop - check navigation status"""
        if self.state == NavigationState.IDLE:
            return
        
        # Check navigation timeout
        if self.state == NavigationState.NAVIGATING and self.goal_handle:
            # Timeout handling can be added here
            pass

    def status_update(self):
        """Periodic status logging"""
        self.get_logger().debug(
            f'Navigation State: {self.state.name}, '
            f'Waypoint: {self.current_waypoint_idx}/{len(self.waypoints)}'
        )

    def get_mission_progress(self) -> dict:
        """Return mission progress information"""
        return {
            'state': self.state.name,
            'current_waypoint': self.current_waypoint_idx,
            'total_waypoints': len(self.waypoints),
            'retry_count': self.retry_count,
        }


def main(args=None):
    rclpy.init(args=args)
    nav_manager = NavigationManager()
    rclpy.spin(nav_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()