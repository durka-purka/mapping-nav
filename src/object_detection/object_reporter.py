"""
Object Reporter for Cube and Landmark Detection

Handles detection, localization, and reporting of:
- Colored cubes (red, green, blue, white)
- Landmarks and their placards

All coordinates reported relative to start frame for compliance.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import json
from enum import Enum
from dataclasses import dataclass, asdict
from typing import List, Optional
import math


class CubeColor(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    WHITE = 3


@dataclass
class CubeDetection:
    """Record of a detected cube"""
    color: str
    pose_in_start_frame: List[float]  # [x, y, z]
    pose_in_map: List[float]
    detection_time: float
    confidence: float
    image_source: str
    autonomous_phase: bool


@dataclass
class LandmarkDetection:
    """Record of a detected landmark"""
    landmark_id: int
    name: str
    placard_text: str
    pose_in_start_frame: List[float]  # [x, y]
    pose_in_map: List[float]
    arrival_time: float
    image_acquired: bool
    autonomous_phase: bool


class ObjectReporter(Node):
    """
    Reports detected objects with proper coordinate transformations.
    
    Listens for detection results from perception module and transforms
    them to the start frame before reporting for compliance.
    """
    
    def __init__(self):
        super().__init__('object_reporter')
        
        # Parameters
        self.declare_parameter('autonomous_phase', True)
        self.declare_parameter('reference_frame', 'start_frame')
        self.declare_parameter('detection_frame', 'map')
        
        self.autonomous_phase = self.get_parameter('autonomous_phase').value
        self.ref_frame = self.get_parameter('reference_frame').value
        self.detect_frame = self.get_parameter('detection_frame').value
        
        # State
        self.detected_cubes: List[CubeDetection] = []
        self.detected_landmarks: List[LandmarkDetection] = []
        self.detection_log = []
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Subscriptions
        self.cube_detection_sub = self.create_subscription(
            String,
            'perception/cube_detections',  # {color, x, y, z, confidence}
            self.cube_detection_callback,
            reliable_qos
        )
        
        self.landmark_detection_sub = self.create_subscription(
            String,
            'perception/landmark_detections',  # {id, text, x, y}
            self.landmark_detection_callback,
            reliable_qos
        )
        
        # Publishers
        self.cube_report_pub = self.create_publisher(
            String,
            'object_reporter/cube_report',
            10
        )
        
        self.landmark_report_pub = self.create_publisher(
            String,
            'object_reporter/landmark_report',
            10
        )
        
        self.get_logger().info('Object Reporter initialized')

    def cube_detection_callback(self, msg: String):
        """Process cube detection message"""
        try:
            detection_data = json.loads(msg.data)
            
            # Extract detection info
            color = detection_data['color']
            pose_map = Pose()
            pose_map.position.x = detection_data['x']
            pose_map.position.y = detection_data['y']
            pose_map.position.z = detection_data['z']
            pose_map.orientation.w = 1.0
            
            # Transform to start frame
            pose_start = self.transform_to_start_frame(pose_map)
            
            # Create detection record
            cube = CubeDetection(
                color=color,
                pose_in_start_frame=[
                    float(pose_start.position.x),
                    float(pose_start.position.y),
                    float(pose_start.position.z)
                ],
                pose_in_map=[
                    float(pose_map.position.x),
                    float(pose_map.position.y),
                    float(pose_map.position.z)
                ],
                detection_time=self.get_clock().now().nanoseconds / 1e9,
                confidence=detection_data.get('confidence', 0.8),
                image_source=detection_data.get('image_source', 'unknown'),
                autonomous_phase=self.autonomous_phase
            )
            
            # Check for duplicates
            is_duplicate = False
            for existing in self.detected_cubes:
                dist = math.sqrt(
                    (cube.pose_in_start_frame[0] - existing.pose_in_start_frame[0])**2 +
                    (cube.pose_in_start_frame[1] - existing.pose_in_start_frame[1])**2
                )
                if dist < 0.2:  # Less than 20cm apart = same cube
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                self.detected_cubes.append(cube)
                self.get_logger().info(
                    f'NEW CUBE DETECTED: {color} at '
                    f'({cube.pose_in_start_frame[0]:.2f}, {cube.pose_in_start_frame[1]:.2f}) '
                    f'in start_frame'
                )
                
                # Publish report
                report = self.create_cube_report(cube)
                self.cube_report_pub.publish(report)
                
                # Log for final report
                self.detection_log.append({
                    'type': 'cube',
                    'data': asdict(cube)
                })
            
        except Exception as e:
            self.get_logger().error(f'Cube detection processing failed: {e}')

    def landmark_detection_callback(self, msg: String):
        """Process landmark detection message"""
        try:
            detection_data = json.loads(msg.data)
            
            # Extract detection info
            landmark_id = detection_data['id']
            pose_map = Pose()
            pose_map.position.x = detection_data['x']
            pose_map.position.y = detection_data['y']
            pose_map.position.z = 0.0
            pose_map.orientation.w = 1.0
            
            # Transform to start frame
            pose_start = self.transform_to_start_frame(pose_map)
            
            # Create detection record
            landmark = LandmarkDetection(
                landmark_id=landmark_id,
                name=detection_data.get('name', f'Landmark_{landmark_id}'),
                placard_text=detection_data.get('placard_text', ''),
                pose_in_start_frame=[
                    float(pose_start.position.x),
                    float(pose_start.position.y)
                ],
                pose_in_map=[
                    float(pose_map.position.x),
                    float(pose_map.position.y)
                ],
                arrival_time=self.get_clock().now().nanoseconds / 1e9,
                image_acquired=detection_data.get('image_acquired', False),
                autonomous_phase=self.autonomous_phase
            )
            
            # Check if landmark already recorded
            is_new = True
            for existing in self.detected_landmarks:
                if existing.landmark_id == landmark.landmark_id:
                    # Update if this is more recent
                    if landmark.arrival_time > existing.arrival_time:
                        self.detected_landmarks.remove(existing)
                    else:
                        is_new = False
                    break
            
            if is_new:
                self.detected_landmarks.append(landmark)
                self.get_logger().info(
                    f'LANDMARK VISITED: {landmark.name} (ID {landmark.landmark_id}) at '
                    f'({landmark.pose_in_start_frame[0]:.2f}, {landmark.pose_in_start_frame[1]:.2f}) '
                    f'in start_frame'
                )
                
                # Publish report
                report = self.create_landmark_report(landmark)
                self.landmark_report_pub.publish(report)
                
                # Log for final report
                self.detection_log.append({
                    'type': 'landmark',
                    'data': asdict(landmark)
                })
            
        except Exception as e:
            self.get_logger().error(f'Landmark detection processing failed: {e}')

    def transform_to_start_frame(self, pose_map: Pose) -> Pose:
        """Transform pose from map to start frame"""
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.detect_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose_map
            
            transform = self.tf_buffer.lookup_transform(
                self.ref_frame,
                self.detect_frame,
                rclpy.time.Time()
            )
            
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return pose_transformed.pose
            
        except Exception as e:
            self.get_logger().warn(f'Transform failed, returning original pose: {e}')
            return pose_map

    def create_cube_report(self, cube: CubeDetection) -> String:
        """Create reportable cube detection message"""
        report = String()
        report.data = json.dumps({
            'type': 'cube_detection',
            'color': cube.color,
            'position_start_frame': cube.pose_in_start_frame,
            'position_map': cube.pose_in_map,
            'detection_time': cube.detection_time,
            'confidence': cube.confidence,
            'autonomous_phase': cube.autonomous_phase,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        })
        return report

    def create_landmark_report(self, landmark: LandmarkDetection) -> String:
        """Create reportable landmark detection message"""
        report = String()
        report.data = json.dumps({
            'type': 'landmark_detection',
            'landmark_id': landmark.landmark_id,
            'name': landmark.name,
            'placard_text': landmark.placard_text,
            'position_start_frame': landmark.pose_in_start_frame,
            'position_map': landmark.pose_in_map,
            'arrival_time': landmark.arrival_time,
            'image_acquired': landmark.image_acquired,
            'autonomous_phase': landmark.autonomous_phase,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        })
        return report

    def get_final_report(self) -> dict:
        """Return final detection report for judges"""
        return {
            'cubes_detected': len(self.detected_cubes),
            'landmarks_visited': len(self.detected_landmarks),
            'detections': self.detection_log,
            'cubes': [asdict(c) for c in self.detected_cubes],
            'landmarks': [asdict(l) for l in self.detected_landmarks],
            'report_generated_at': self.get_clock().now().nanoseconds / 1e9
        }


def main(args=None):
    rclpy.init(args=args)
    reporter = ObjectReporter()
    rclpy.spin(reporter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()