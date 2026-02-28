"""
Waypoint Handler for Australian Rover Challenge 2026
Activity 2: Autonomous Landmark Navigation

Purpose:
- Load landmark locations from task schematic
- Queue waypoints in optimal order
- Detect landmark arrival
- Trigger placard detection & imaging
- Track navigation progress and success

Reference frame: /map (RTAB-Map output)
Target frame: /base_link (rover center)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Int32
from std_srvs.srv import Trigger

from tf2_ros import TransformListener, Buffer, LookupException
import tf_transformations

import yaml
import json
import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime


class LandmarkState(Enum):
    """State machine for landmark processing"""
    UNVISITED = 0
    NAVIGATING = 1
    ARRIVED = 2
    IMAGING = 3
    COMPLETED = 4
    FAILED = 5


@dataclass
class Landmark:
    """Data structure for landmark information"""
    id: int
    name: str
    position_x: float      # meters in map frame
    position_y: float
    height_above_ground: float  # meters (for placard height)
    placard_text: str = ""  # Detected text content
    image_timestamp: Optional[float] = None
    visit_timestamp: Optional[float] = None
    state: LandmarkState = LandmarkState.UNVISITED
    retry_count: int = 0
    max_retries: int = 3


class WaypointHandler(Node):
    """
    Orchestrates landmark-based autonomous navigation
    
    Inputs:
    - Task schematic (JSON/YAML with landmark locations)
    - Current robot pose (from EKF localization)
    - Placard detection results
    
    Outputs:
    - Waypoint goals to Nav2
    - Placard imaging commands
    - Landmark visitation reports
    """
    
    def __init__(self):
        super().__init__('waypoint_handler')
        
        # Parameters
        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('arrival_distance_threshold', 0.3)  # meters
        self.declare_parameter('arrival_angle_threshold', 0.3)     # radians (~17°)
        self.declare_parameter('imaging_distance', 0.5)            # meters from landmark
        self.declare_parameter('timeout_per_landmark', 120)        # seconds
        self.declare_parameter('nav_attempt_timeout', 60)          # seconds per attempt
        
        self.landmarks_file = self.get_parameter('landmarks_file').value
        self.arrival_distance = self.get_parameter('arrival_distance_threshold').value
        self.arrival_angle = self.get_parameter('arrival_angle_threshold').value
        self.imaging_distance = self.get_parameter('imaging_distance').value
        self.timeout_landmark = self.get_parameter('timeout_per_landmark').value
        self.timeout_nav_attempt = self.get_parameter('nav_attempt_timeout').value
        
        # Landmarks database
        self.landmarks: List[Landmark] = []
        self.current_landmark_idx = 0
        self.autonomous_mode = True
        
        # TF2 for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.goal_start_time = None
        
        # Robot state
        self.current_pose = None
        self.current_odom = None
        
        # QoS for reliable communication
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',  # EKF-fused odometry from robot_localization
            self.odom_callback,
            reliable_qos
        )
        
        self.placard_detection_sub = self.create_subscription(
            String,
            'perception/placard_text',  # From placard detector
            self.placard_callback,
            reliable_qos
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'waypoint_handler/status', 10)
        self.current_landmark_pub = self.create_publisher(Int32, 'waypoint_handler/current_landmark', 10)
        
        # Service client for placard imaging
        self.image_placard_client = self.create_client(
            Trigger,
            'perception/image_placard'
        )
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Timers
        self.create_timer(0.5, self.update_loop, callback_group=self.callback_group)
        self.create_timer(1.0, self.status_update, callback_group=self.callback_group)
        
        # Load landmarks from file
        self.load_landmarks()
        
        self.get_logger().info(f'Waypoint Handler initialized with {len(self.landmarks)} landmarks')

    def load_landmarks(self) -> bool:
        """
        Load landmark locations from task schematic file
        
        File format (YAML):
        landmarks:
          - id: 0
            name: "Landmark A"
            position_x: 1.5
            position_y: 2.3
            height_above_ground: 0.8
          - id: 1
            ...
        """
        try:
            with open(self.landmarks_file, 'r') as f:
                data = yaml.safe_load(f)
            
            self.landmarks = []
            for lm_data in data.get('landmarks', []):
                landmark = Landmark(
                    id=lm_data['id'],
                    name=lm_data.get('name', f"Landmark_{lm_data['id']}"),
                    position_x=lm_data['position_x'],
                    position_y=lm_data['position_y'],
                    height_above_ground=lm_data.get('height_above_ground', 0.8),
                )
                self.landmarks.append(landmark)
            
            # Sort by nearest distance from start
            self.landmarks.sort(
                key=lambda lm: math.sqrt(lm.position_x**2 + lm.position_y**2)
            )
            
            self.get_logger().info(f'Loaded {len(self.landmarks)} landmarks from {self.landmarks_file}')
            return True
            
        except FileNotFoundError:
            self.get_logger().error(f'Landmarks file not found: {self.landmarks_file}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error loading landmarks: {e}')
            return False

    def odom_callback(self, msg: Odometry):
        """Update current pose from EKF odometry"""
        self.current_odom = msg
        self.current_pose = msg.pose.pose

    def placard_callback(self, msg: String):
        """Store detected placard text for current landmark"""
        if self.current_landmark_idx < len(self.landmarks):
            self.landmarks[self.current_landmark_idx].placard_text = msg.data
            self.get_logger().info(
                f'Placard detected for landmark {self.current_landmark_idx}: {msg.data}'
            )

    def start_autonomous_navigation(self) -> bool:
        """Initialize autonomous navigation sequence"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False
        
        self.autonomous_mode = True
        self.current_landmark_idx = 0
        self.get_logger().info('Starting autonomous landmark navigation')
        
        return self.navigate_to_next_landmark()

    def navigate_to_next_landmark(self) -> bool:
        """Send next unvisited landmark to Nav2"""
        # Find next unvisited landmark
        while self.current_landmark_idx < len(self.landmarks):
            lm = self.landmarks[self.current_landmark_idx]
            if lm.state == LandmarkState.UNVISITED:
                break
            self.current_landmark_idx += 1
        
        if self.current_landmark_idx >= len(self.landmarks):
            self.get_logger().info('All landmarks visited')
            return False
        
        landmark = self.landmarks[self.current_landmark_idx]
        
        # Create navigation goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = landmark.position_x
        goal.pose.pose.position.y = landmark.position_y
        goal.pose.pose.position.z = 0.0
        
        # Orientation: face away from start (arbitrary heading)
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        goal.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        
        self.get_logger().info(
            f'Navigating to landmark {self.current_landmark_idx}: {landmark.name} '
            f'({landmark.position_x:.2f}, {landmark.position_y:.2f})'
        )
        
        landmark.state = LandmarkState.NAVIGATING
        landmark.retry_count += 1
        self.goal_start_time = self.get_clock().now()
        
        # Send goal asynchronously
        self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_callback
        ).add_done_callback(self.nav_goal_response_callback)
        
        return True

    def nav_feedback_callback(self, msg):
        """Handle Nav2 feedback during navigation"""
        remaining_distance = msg.feedback.navigation_feedback.remaining_distance
        self.get_logger().debug(f'Distance to landmark: {remaining_distance:.2f}m')

    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance/rejection"""
        self.current_goal_handle = future.result()
        
        if not self.current_goal_handle.accepted:
            self.get_logger().warning('Navigation goal rejected')
            self.handle_navigation_failure()
            return
        
        # Wait for result
        self.current_goal_handle.get_result_async().add_done_callback(
            self.nav_result_callback
        )

    def nav_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result()
        
        if result.result:
            self.get_logger().info(
                f'Reached landmark {self.current_landmark_idx}'
            )
            self.landmarks[self.current_landmark_idx].state = LandmarkState.ARRIVED
            self.landmarks[self.current_landmark_idx].visit_timestamp = \
                self.get_clock().now().nanoseconds / 1e9
            
            # Trigger placard imaging
            self.image_placard()
        else:
            self.handle_navigation_failure()

    def handle_navigation_failure(self):
        """Handle failed navigation attempt"""
        landmark = self.landmarks[self.current_landmark_idx]
        
        if landmark.retry_count >= landmark.max_retries:
            self.get_logger().warning(
                f'Max retries exceeded for landmark {self.current_landmark_idx}'
            )
            landmark.state = LandmarkState.FAILED
            self.autonomous_mode = False  # Trigger intervention
        else:
            self.get_logger().info(f'Retrying landmark {self.current_landmark_idx}')
            self.navigate_to_next_landmark()

    def image_placard(self):
        """Request placard imaging from perception module"""
        self.get_logger().info(f'Requesting placard image for landmark {self.current_landmark_idx}')
        
        if not self.image_placard_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('Placard imaging service not available')
            self.continue_to_next_landmark()
            return
        
        request = Trigger.Request()
        future = self.image_placard_client.call_async(request)
        future.add_done_callback(self.image_placard_callback)

    def image_placard_callback(self, future):
        """Handle placard imaging completion"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Placard image captured for landmark {self.current_landmark_idx}')
                self.landmarks[self.current_landmark_idx].state = LandmarkState.IMAGING
                self.landmarks[self.current_landmark_idx].image_timestamp = \
                    self.get_clock().now().nanoseconds / 1e9
            else:
                self.get_logger().warning(f'Placard imaging failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Placard imaging error: {e}')
        
        self.continue_to_next_landmark()

    def continue_to_next_landmark(self):
        """Move to next landmark in sequence"""
        self.landmarks[self.current_landmark_idx].state = LandmarkState.COMPLETED
        self.current_landmark_idx += 1
        
        if self.current_landmark_idx < len(self.landmarks):
            self.navigate_to_next_landmark()
        else:
            self.get_logger().info('All landmarks visited - switching to exploration mode')
            self.autonomous_mode = False

    def update_loop(self):
        """Main update loop - check navigation progress"""
        if not self.autonomous_mode or not self.current_goal_handle:
            return
        
        # Check timeout
        if self.goal_start_time:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed > self.timeout_nav_attempt:
                self.get_logger().warning(
                    f'Navigation timeout for landmark {self.current_landmark_idx}'
                )
                self.handle_navigation_failure()
        
        # Check arrival
        landmark = self.landmarks[self.current_landmark_idx]
        if self.current_pose and self.check_arrival(landmark):
            if self.current_goal_handle.get_status() not in [4, 5]:  # Not completed
                self.get_logger().info(
                    f'Landmark {self.current_landmark_idx} within arrival threshold'
                )

    def check_arrival(self, landmark: Landmark) -> bool:
        """Check if rover has arrived at landmark"""
        if not self.current_pose:
            return False
        
        # Distance check
        dx = self.current_pose.position.x - landmark.position_x
        dy = self.current_pose.position.y - landmark.position_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.arrival_distance:
            return True
        
        return False

    def status_update(self):
        """Publish status updates"""
        status_msg = String()
        status_data = {
            'autonomous_mode': self.autonomous_mode,
            'current_landmark': self.current_landmark_idx,
            'total_landmarks': len(self.landmarks),
            'landmarks': [
                {
                    'id': lm.id,
                    'name': lm.name,
                    'state': lm.state.name,
                    'placard_text': lm.placard_text,
                    'retry_count': lm.retry_count,
                }
                for lm in self.landmarks
            ]
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
        
        self.current_landmark_pub.publish(Int32(data=self.current_landmark_idx))

    def get_mission_progress(self) -> dict:
        """Return detailed mission progress"""
        completed = sum(1 for lm in self.landmarks if lm.state == LandmarkState.COMPLETED)
        failed = sum(1 for lm in self.landmarks if lm.state == LandmarkState.FAILED)
        
        return {
            'total_landmarks': len(self.landmarks),
            'completed': completed,
            'failed': failed,
            'in_progress': self.current_landmark_idx,
            'landmarks': [
                {
                    'id': lm.id,
                    'name': lm.name,
                    'state': lm.state.name,
                    'placard_text': lm.placard_text,
                    'timestamp': lm.image_timestamp,
                }
                for lm in self.landmarks
            ]
        }


def main(args=None):
    rclpy.init(args=args)
    waypoint_handler = WaypointHandler()
    rclpy.spin(waypoint_handler)
    rclpy.shutdown()


if __name__ == '__main__':
    main()