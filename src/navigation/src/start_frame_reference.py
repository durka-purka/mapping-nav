"""
Start Frame Reference Manager

Manages coordinate transformation from map frame to rover start frame.
All cube and landmark coordinates are reported relative to this frame.

Rule: All rover outputs must be relative to starting position/frame.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import tf_transformations
import tf2_geometry_msgs
from std_msgs.msg import String
from std_srvs.srv import SetBool
import json
from enum import Enum


class StartFrameReference(Node):
    """
    Manages the coordinate reference frame for all rover outputs.
    
    At task start (before first movement):
    1. Capture the initial robot pose in the map frame
    2. Create a static transform (map → start_frame)
    3. All cube/landmark/obstacle coordinates are transformed to start_frame
    
    This ensures all outputs are "relative to starting position" as required.
    """
    
    def __init__(self):
        super().__init__('start_frame_reference')
        
        # Parameters
        self.declare_parameter('reference_frame_id', 'start_frame')
        self.declare_parameter('robot_initial_frame_id', 'map')
        self.declare_parameter('auto_initialize', True)
        
        self.ref_frame = self.get_parameter('reference_frame_id').value
        self.robot_frame = self.get_parameter('robot_initial_frame_id').value
        self.auto_init = self.get_parameter('auto_initialize').value
        
        # State
        self.start_pose_map = None  # Robot pose in map frame at start
        self.map_to_start_transform = None  # Transform map → start_frame
        self.initialized = False
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscriptions
        self.initial_pose_sub = self.create_subscription(
            PoseStamped,
            '/initialpose',  # From RViz or initial localization
            self.initial_pose_callback,
            10
        )
        
        # Services
        self.initialize_service = self.create_service(
            SetBool,
            'start_frame_reference/initialize',
            self.initialize_start_frame
        )
        
        # Publishers
        self.initialized_pub = self.create_publisher(
            String,
            'start_frame_reference/status',
            10
        )
        
        # Timer for auto-init if enabled
        if self.auto_init:
            self.create_timer(1.0, self.auto_initialize_check)
        
        self.get_logger().info('Start Frame Reference Manager initialized')

    def initial_pose_callback(self, msg: PoseStamped):
        """
        Capture the initial pose (usually from localization initialization or RViz)
        """
        self.get_logger().info(f'Received initial pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.start_pose_map = msg.pose

    def auto_initialize_check(self):
        """Check if we should auto-initialize (when localization is ready)"""
        if self.initialized:
            return
        
        try:
            # Try to get current pose from tf2
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1)
            )
            
            # Extract pose from transform
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            self.start_pose_map = pose
            self.initialize_start_frame_internal()
            
        except Exception as e:
            # Not ready yet
            pass

    def initialize_start_frame(self, request: SetBool.Request, response: SetBool.Response):
        """Service to manually initialize start frame"""
        try:
            self.initialize_start_frame_internal()
            response.success = True
            response.message = 'Start frame initialized'
            return response
        except Exception as e:
            response.success = False
            response.message = str(e)
            return response

    def initialize_start_frame_internal(self):
        """Initialize the start frame transform"""
        if self.initialized:
            self.get_logger().warn('Start frame already initialized')
            return
        
        if self.start_pose_map is None:
            raise RuntimeError('No initial pose captured yet')
        
        # Create transform: map → start_frame
        # The start frame is at the robot's initial position in the map
        # So the transform translates and rotates points from map to start frame
        
        # Translation component: negate the initial pose
        trans = [
            -self.start_pose_map.position.x,
            -self.start_pose_map.position.y,
            -self.start_pose_map.position.z
        ]
        
        # Rotation component: inverse of initial orientation
        quat = [
            self.start_pose_map.orientation.x,
            self.start_pose_map.orientation.y,
            self.start_pose_map.orientation.z,
            self.start_pose_map.orientation.w
        ]
        
        # Get inverse rotation
        quat_inverse = tf_transformations.quaternion_conjugate(quat)
        
        # Rotate translation
        trans_rotated = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(quat_inverse, [trans[0], trans[1], trans[2], 0]),
            quat
        )[:3]
        
        # Create and broadcast static transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.robot_frame  # map
        transform.child_frame_id = self.ref_frame  # start_frame
        
        transform.transform.translation.x = trans_rotated[0]
        transform.transform.translation.y = trans_rotated[1]
        transform.transform.translation.z = trans_rotated[2]
        
        transform.transform.rotation.x = quat_inverse[0]
        transform.transform.rotation.y = quat_inverse[1]
        transform.transform.rotation.z = quat_inverse[2]
        transform.transform.rotation.w = quat_inverse[3]
        
        self.static_tf_broadcaster.sendTransform(transform)
        
        self.initialized = True
        self.map_to_start_transform = transform
        
        self.get_logger().info(
            f'Start frame initialized: {self.ref_frame} at '
            f'({self.start_pose_map.position.x:.2f}, {self.start_pose_map.position.y:.2f}) in {self.robot_frame}'
        )
        
        # Publish status
        status_msg = String()
        status_data = {
            'initialized': True,
            'start_pose_in_map': [
                float(self.start_pose_map.position.x),
                float(self.start_pose_map.position.y)
            ],
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        status_msg.data = json.dumps(status_data)
        self.initialized_pub.publish(status_msg)

    def transform_pose_to_start_frame(self, pose_in_map: Pose) -> Pose:
        """
        Transform a pose from map frame to start frame.
        
        Args:
            pose_in_map: Pose in map frame
            
        Returns:
            Pose in start_frame
        """
        if not self.initialized:
            raise RuntimeError('Start frame not initialized yet')
        
        # Create PoseStamped for transformation
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.robot_frame
        pose_stamped.pose = pose_in_map
        
        try:
            # Get the transform
            transform = self.tf_buffer.lookup_transform(
                self.ref_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            
            # Apply transform
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return pose_transformed.pose
            
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return pose_in_map


def main(args=None):
    rclpy.init(args=args)
    manager = StartFrameReference()
    rclpy.spin(manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()