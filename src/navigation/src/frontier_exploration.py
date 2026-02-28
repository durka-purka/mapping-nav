"""
Frontier-Based Exploration for Activity 3
Australian Rover Challenge 2026

Purpose:
- Detect frontier (known/unknown boundary) in costmap
- Select optimal frontier based on information gain
- Send frontier centroids as navigation goals
- Handle cube detection during exploration

Frontier detection is critical for exploring the arena and finding the 4 colored cubes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray, Marker

import numpy as np
from scipy import ndimage
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional
import math
import time


class ExplorationState(Enum):
    """Exploration state machine"""
    IDLE = 0
    SCANNING_FRONTIERS = 1
    NAVIGATING_TO_FRONTIER = 2
    DETECTING_OBJECTS = 3
    COMPLETED = 4


@dataclass
class FrontierCluster:
    """Represents a frontier region"""
    centroid_x: float
    centroid_y: float
    size: int  # Number of frontier cells
    distance_from_robot: float
    information_gain: float  # Higher = more unknown territory nearby
    priority: float  # Composite score for selection


class FrontierExplorer(Node):
    """
    Detects and navigates to frontier regions for exploration
    
    Algorithm:
    1. Subscribe to costmap from Nav2
    2. Identify frontier cells (borders between known and unknown)
    3. Cluster frontier cells using connected components
    4. Score clusters by distance and information gain
    5. Navigate to highest-priority frontier
    6. Repeat until costmap is fully explored
    
    Integration with cube detection:
    - When arriving at frontier, trigger cube detector
    - Store cube detections in global frame
    """
    
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Parameters
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('min_frontier_size', 10)  # minimum cells in cluster
        self.declare_parameter('exploration_frequency', 1.0)  # Hz
        self.declare_parameter('max_exploration_time', 600)  # seconds
        self.declare_parameter('cube_detection_enabled', True)
        self.declare_parameter('visualize_frontiers', True)
        
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_freq = self.get_parameter('exploration_frequency').value
        self.max_exploration_time = self.get_parameter('max_exploration_time').value
        self.cube_detection_enabled = self.get_parameter('cube_detection_enabled').value
        self.visualize_frontiers = self.get_parameter('visualize_frontiers').value
        
        # State
        self.state = ExplorationState.IDLE
        self.costmap = None
        self.current_pose = None
        self.frontier_clusters: List[FrontierCluster] = []
        self.navigation_start_time = None
        self.discovered_objects = []
        
        # Navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        
        # Subscriptions
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            reliable_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            reliable_qos
        )
        
        self.cube_detection_sub = self.create_subscription(
            String,
            'perception/cube_detections',  # JSON: {color, x, y, z}
            self.cube_callback,
            reliable_qos
        )
        
        # Publishers
        self.frontier_markers_pub = self.create_publisher(
            MarkerArray,
            'frontier_explorer/frontier_markers',
            10
        )
        
        self.exploration_status_pub = self.create_publisher(
            String,
            'frontier_explorer/status',
            10
        )
        
        # Service client for cube detection
        self.detect_cubes_client = self.create_client(
            Trigger,
            'perception/detect_cubes'
        )
        
        # Callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Timers
        self.create_timer(
            1.0 / self.exploration_freq,
            self.exploration_loop,
            callback_group=self.callback_group
        )
        self.create_timer(2.0, self.status_update, callback_group=self.callback_group)
        
        self.get_logger().info('Frontier Explorer initialized')

    def costmap_callback(self, msg: OccupancyGrid):
        """Store costmap for frontier detection"""
        self.costmap = msg

    def odom_callback(self, msg: Odometry):
        """Update current pose"""
        self.current_pose = msg.pose.pose

    def cube_callback(self, msg: String):
        """Store detected cube location"""
        import json
        try:
            cube_data = json.loads(msg.data)
            cube_data['timestamp'] = self.get_clock().now().nanoseconds / 1e9
            self.discovered_objects.append(cube_data)
            
            self.get_logger().info(
                f"Cube detected: {cube_data['color']} at "
                f"({cube_data['x']:.2f}, {cube_data['y']:.2f}, {cube_data['z']:.2f})"
            )
        except Exception as e:
            self.get_logger().error(f'Error parsing cube detection: {e}')

    def start_exploration(self) -> bool:
        """Initialize frontier exploration"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False
        
        self.state = ExplorationState.SCANNING_FRONTIERS
        self.navigation_start_time = self.get_clock().now()
        self.get_logger().info('Starting frontier exploration')
        return True

    def exploration_loop(self):
        """Main exploration loop"""
        if self.state == ExplorationState.IDLE:
            return
        
        # Check exploration timeout
        if self.navigation_start_time:
            elapsed = (self.get_clock().now() - self.navigation_start_time).nanoseconds / 1e9
            if elapsed > self.max_exploration_time:
                self.get_logger().info('Exploration time limit reached')
                self.state = ExplorationState.COMPLETED
                return
        
        # Detect frontiers in costmap
        if self.costmap:
            self.detect_frontiers()
            
            # If no frontiers, exploration is complete
            if not self.frontier_clusters:
                self.get_logger().info('No more frontiers detected - exploration complete')
                self.state = ExplorationState.COMPLETED
                return
            
            # Visualize frontiers
            if self.visualize_frontiers:
                self.publish_frontier_markers()
            
            # Navigate to next frontier
            if self.state == ExplorationState.SCANNING_FRONTIERS:
                self.navigate_to_frontier()

    def detect_frontiers(self):
        """
        Detect frontier cells in occupancy grid
        
        Frontier cells are those that:
        - Are known (free space, cost = 0)
        - Neighbor unknown space (cost = -1)
        
        Uses connected components to cluster frontiers.
        """
        if not self.costmap:
            return
        
        # Convert OccupancyGrid to numpy array
        width = self.costmap.info.width
        height = self.costmap.info.height
        grid = np.array(self.costmap.data, dtype=np.int8).reshape((height, width))
        
        # Find frontier cells
        # -1 = unknown, 0 = free, 100 = occupied
        frontier_grid = np.zeros_like(grid, dtype=np.uint8)
        
        # Interior cells (not at boundary)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if grid[y, x] == 0:  # Free cell
                    # Check neighbors for unknown
                    neighbors = grid[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors:
                        frontier_grid[y, x] = 1
        
        # Connected component labeling
        labeled_array, num_features = ndimage.label(frontier_grid)
        
        # Extract cluster properties
        self.frontier_clusters = []
        
        for cluster_id in range(1, num_features + 1):
            cluster_mask = labeled_array == cluster_id
            cluster_size = np.sum(cluster_mask)
            
            # Filter by minimum size
            if cluster_size < self.min_frontier_size:
                continue
            
            # Get cluster centroid in grid coordinates
            y_coords, x_coords = np.where(cluster_mask)
            centroid_grid_x = np.mean(x_coords)
            centroid_grid_y = np.mean(y_coords)
            
            # Convert to world coordinates
            centroid_world_x = self.costmap.info.origin.position.x + \
                              centroid_grid_x * self.costmap.info.resolution
            centroid_world_y = self.costmap.info.origin.position.y + \
                              centroid_grid_y * self.costmap.info.resolution
            
            # Calculate information gain (number of unknown cells nearby)
            info_gain = self.calculate_information_gain(cluster_mask, labeled_array)
            
            # Distance to robot
            if self.current_pose:
                dx = centroid_world_x - self.current_pose.position.x
                dy = centroid_world_y - self.current_pose.position.y
                distance = math.sqrt(dx**2 + dy**2)
            else:
                distance = float('inf')
            
            # Priority score (weighted combination)
            # Higher information gain = more priority
            # Closer distance = more priority
            priority = (info_gain * 0.6) - (distance * 0.4)
            
            cluster = FrontierCluster(
                centroid_x=centroid_world_x,
                centroid_y=centroid_world_y,
                size=int(cluster_size),
                distance_from_robot=distance,
                information_gain=info_gain,
                priority=priority
            )
            
            self.frontier_clusters.append(cluster)
        
        # Sort by priority
        self.frontier_clusters.sort(key=lambda c: c.priority, reverse=True)
        
        self.get_logger().debug(f'Detected {len(self.frontier_clusters)} frontier clusters')

    def calculate_information_gain(self, cluster_mask: np.ndarray, 
                                   labeled_array: np.ndarray) -> float:
        """
        Calculate information gain for a frontier cluster
        
        Metric: ratio of unknown cells adjacent to frontier
        """
        height, width = cluster_mask.shape
        unknown_neighbors = 0
        total_neighbors = 0
        
        y_coords, x_coords = np.where(cluster_mask)
        
        for y, x in zip(y_coords, x_coords):
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dy == 0 and dx == 0:
                        continue
                    
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        total_neighbors += 1
                        # Check if unknown (-1) but not in current cluster
                        if labeled_array[ny, nx] == 0:  # Not labeled (unknown or occupied)
                            # Additional check if actually unknown
                            unknown_neighbors += 1
        
        if total_neighbors == 0:
            return 0.0
        
        return float(unknown_neighbors) / float(total_neighbors)

    def navigate_to_frontier(self):
        """Send navigation goal to best frontier"""
        if not self.frontier_clusters:
            self.get_logger().warning('No frontier clusters available')
            return
        
        best_frontier = self.frontier_clusters[0]
        
        # Create navigation goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = best_frontier.centroid_x
        goal.pose.pose.position.y = best_frontier.centroid_y
        goal.pose.pose.position.z = 0.0
        
        # Orientation (face towards frontier direction)
        from geometry_msgs.msg import Quaternion
        import tf_transformations
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        goal.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        
        self.get_logger().info(
            f'Navigating to frontier at ({best_frontier.centroid_x:.2f}, '
            f'{best_frontier.centroid_y:.2f}), priority={best_frontier.priority:.2f}'
        )
        
        self.state = ExplorationState.NAVIGATING_TO_FRONTIER
        
        # Send goal
        self.nav_client.send_goal_async(goal).add_done_callback(
            self.frontier_goal_response_callback
        )

    def frontier_goal_response_callback(self, future):
        """Handle frontier navigation response"""
        self.current_goal_handle = future.result()
        
        if not self.current_goal_handle.accepted:
            self.get_logger().warning('Frontier goal rejected')
            return
        
        self.current_goal_handle.get_result_async().add_done_callback(
            self.frontier_result_callback
        )

    def frontier_result_callback(self, future):
        """Handle frontier navigation result"""
        result = future.result()
        
        if result.result:
            self.get_logger().info('Reached frontier')
            
            # Trigger cube detection
            if self.cube_detection_enabled:
                self.detect_cubes_at_frontier()
            else:
                self.state = ExplorationState.SCANNING_FRONTIERS
        else:
            self.get_logger().warning('Failed to reach frontier')
            self.state = ExplorationState.SCANNING_FRONTIERS

    def detect_cubes_at_frontier(self):
        """Request cube detection at current location"""
        self.state = ExplorationState.DETECTING_OBJECTS
        
        if not self.detect_cubes_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('Cube detection service not available')
            self.state = ExplorationState.SCANNING_FRONTIERS
            return
        
        request = Trigger.Request()
        future = self.detect_cubes_client.call_async(request)
        future.add_done_callback(self.cube_detection_callback)

    def cube_detection_callback(self, future):
        """Handle cube detection completion"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Cube detection completed')
            else:
                self.get_logger().warning(f'Cube detection failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Cube detection error: {e}')
        
        # Continue scanning for more frontiers
        self.state = ExplorationState.SCANNING_FRONTIERS

    def publish_frontier_markers(self):
        """Publish frontier clusters as RViz markers"""
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(self.frontier_clusters):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier.centroid_x
            marker.pose.position.y = frontier.centroid_y
            marker.pose.position.z = 0.2
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        self.frontier_markers_pub.publish(marker_array)

    def status_update(self):
        """Publish exploration status"""
        status_msg = String()
        status_data = {
            'state': self.state.name,
            'frontier_clusters': len(self.frontier_clusters),
            'discovered_objects': len(self.discovered_objects),
            'objects': self.discovered_objects
        }
        import json
        status_msg.data = json.dumps(status_data)
        self.exploration_status_pub.publish(status_msg)

    def get_exploration_results(self) -> dict:
        """Return exploration results"""
        return {
            'state': self.state.name,
            'discovered_cubes': self.discovered_objects,
            'exploration_time': (
                (self.get_clock().now() - self.navigation_start_time).nanoseconds / 1e9
                if self.navigation_start_time else 0
            )
        }


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    rclpy.spin(explorer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()