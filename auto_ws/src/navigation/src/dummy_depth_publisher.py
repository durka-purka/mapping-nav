"""
Dummy Depth Camera Publisher for Gazebo Testing
Simulates RealSense D435 depth camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DummyDepthPublisher(Node):
    """Publishes dummy depth images"""
    
    def __init__(self):
        super().__init__('dummy_depth_publisher')
        
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image_rect_raw',
            10
        )
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.067, self.publish_depth)  # ~15 Hz
        
        self.get_logger().info('Dummy Depth Publisher started')
    
    def publish_depth(self):
        """Publish dummy depth image"""
        # Create dummy depth image (640x480, random noise)
        depth_array = np.random.randint(300, 2000, (480, 640), dtype=np.uint16)
        
        msg = self.bridge.cv2_to_imgmsg(depth_array, encoding='16UC1')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_optical_frame'
        
        self.depth_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyDepthPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()