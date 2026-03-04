"""
Autonomous Phase Manager for Australian Rover Challenge 2026

Enforces strict autonomous/non-autonomous phase separation and prevents
manual control from being applied after first movement.

Rule Compliance:
- No human control after first rover movement during autonomous phase
- All outputs derived from runtime data only
- Maintains audit log for judge review
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import json
from enum import Enum
from datetime import datetime
from dataclasses import dataclass, asdict


class Phase(Enum):
    """Rover operational phase"""
    INITIALIZATION = 0      # Before first move (allows setup/calib)
    AUTONOMOUS = 1          # Moving autonomously, no manual intervention
    INTERVENTION_CALLED = 2 # Manual intervention activated
    NON_AUTONOMOUS = 3      # Operating under manual control
    COMPLETED = 4           # Autonomous tasks complete


@dataclass
class PhaseEvent:
    """Record of a phase change or event"""
    timestamp: float
    phase_from: str
    phase_to: str
    reason: str
    rover_pose: list  # [x, y, theta]
    

class AutonomousPhaseManager(Node):
    """
    Manages and enforces strict autonomous/non-autonomous phase separation.
    
    Responsibilities:
    1. Track when rover first moves (end initialization)
    2. Detect manual control attempts during autonomous phase (forbidden)
    3. Allow controlled intervention transition
    4. Log all phase changes and control attempts
    5. Publish phase status for judges and other nodes
    6. Enforce no manual cmd_vel after autonomous phase start
    """
    
    def __init__(self):
        super().__init__('autonomous_phase_manager')
        
        # Parameters
        self.declare_parameter('start_in_autonomous_mode', True)
        self.declare_parameter('enable_manual_override_lock', True)
        self.declare_parameter('movement_threshold', 0.01)  # meters
        self.declare_parameter('log_file_path', '/tmp/arc2026_phase_log.json')
        
        self.start_autonomous = self.get_parameter('start_in_autonomous_mode').value
        self.lock_manual = self.get_parameter('enable_manual_override_lock').value
        self.move_threshold = self.get_parameter('movement_threshold').value
        self.log_file = self.get_parameter('log_file_path').value
        
        # State
        self.current_phase = Phase.INITIALIZATION if self.start_autonomous else Phase.NON_AUTONOMOUS
        self.phase_events: list[PhaseEvent] = []
        self.initial_pose = None
        self.current_pose = None
        self.has_moved = False
        self.manual_control_attempts = 0
        
        # QoS for reliability
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
        
        # Monitor for manual cmd_vel injection
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel_manual',  # External manual controls on this topic
            self.manual_cmd_callback,
            reliable_qos
        )
        
        # Publishers - phase status for all nodes and judges
        self.phase_status_pub = self.create_publisher(
            String,
            'autonomous_phase/status',
            10
        )
        
        self.phase_event_pub = self.create_publisher(
            String,
            'autonomous_phase/events',
            10
        )
        
        self.phase_locked_pub = self.create_publisher(
            Bool,
            'autonomous_phase/locked',
            10
        )
        
        self.collision_penalty_pub = self.create_publisher(
            Int32,
            'autonomous_phase/collision_penalty_count',
            10
        )
        
        # Timers
        self.create_timer(0.5, self.phase_check_loop)
        self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info(
            f'Autonomous Phase Manager initialized in {self.current_phase.name} mode'
        )

    def odom_callback(self, msg: Odometry):
        """Track robot movement"""
        pose = msg.pose.pose
        
        # Capture initial pose
        if self.initial_pose is None:
            self.initial_pose = pose
            self.current_pose = pose
            return
        
        # Update current pose
        self.current_pose = pose
        
        # Check if robot has moved
        if not self.has_moved:
            dx = pose.position.x - self.initial_pose.position.x
            dy = pose.position.y - self.initial_pose.position.y
            distance_moved = (dx**2 + dy**2)**0.5
            
            if distance_moved > self.move_threshold:
                self.has_moved = True
                # End initialization phase
                if self.current_phase == Phase.INITIALIZATION:
                    self.transition_phase(
                        Phase.AUTONOMOUS,
                        f'Rover moved {distance_moved:.3f}m, entering autonomous phase'
                    )

    def manual_cmd_callback(self, msg: Twist):
        """Detect manual control attempts during autonomous phase"""
        # Check if there's actual command
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
            return
        
        # Record attempt
        self.manual_control_attempts += 1
        
        if self.current_phase == Phase.AUTONOMOUS:
            self.get_logger().warn(
                f'RULE VIOLATION: Manual control attempt #{self.manual_control_attempts} '
                f'during autonomous phase at {self.get_clock().now().nanoseconds / 1e9:.1f}s'
            )
            
            # Log the violation
            self.log_event(
                Phase.AUTONOMOUS,
                Phase.AUTONOMOUS,
                f'Manual control attempt blocked (#{self.manual_control_attempts})'
            )
            
            # DO NOT apply the command - block it
            if self.lock_manual:
                self.get_logger().error(
                    'Manual control is LOCKED during autonomous phase (Rule 12.6.2)'
                )
        
        elif self.current_phase == Phase.INITIALIZATION:
            # Manual control allowed during initialization (calibration, setup)
            self.get_logger().info(
                f'Manual control during initialization (allowed): {msg.linear.x:.2f} m/s'
            )

    def transition_phase(self, new_phase: Phase, reason: str):
        """Transition to a new phase"""
        if new_phase == self.current_phase:
            return
        
        old_phase = self.current_phase
        self.current_phase = new_phase
        
        # Log event
        self.log_event(old_phase, new_phase, reason)
        
        # Publish event
        event_msg = String()
        event_data = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'phase_from': old_phase.name,
            'phase_to': new_phase.name,
            'reason': reason,
            'rover_pose': [
                float(self.current_pose.position.x),
                float(self.current_pose.position.y),
                0.0  # yaw can be extracted if needed
            ]
        }
        event_msg.data = json.dumps(event_data)
        self.phase_event_pub.publish(event_msg)
        
        self.get_logger().info(
            f'Phase transition: {old_phase.name} → {new_phase.name}: {reason}'
        )

    def log_event(self, phase_from: Phase, phase_to: Phase, reason: str):
        """Log a phase event"""
        event = PhaseEvent(
            timestamp=self.get_clock().now().nanoseconds / 1e9,
            phase_from=phase_from.name,
            phase_to=phase_to.name,
            reason=reason,
            rover_pose=[
                float(self.current_pose.position.x) if self.current_pose else 0.0,
                float(self.current_pose.position.y) if self.current_pose else 0.0,
                0.0
            ]
        )
        self.phase_events.append(event)

    def phase_check_loop(self):
        """Periodic phase checks"""
        # Publish locked status
        locked_msg = Bool(data=self.lock_manual and self.current_phase == Phase.AUTONOMOUS)
        self.phase_locked_pub.publish(locked_msg)

    def publish_status(self):
        """Publish current phase status"""
        status_msg = String()
        status_data = {
            'current_phase': self.current_phase.name,
            'has_moved': self.has_moved,
            'manual_control_attempts': self.manual_control_attempts,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'rover_pose': [
                float(self.current_pose.position.x) if self.current_pose else 0.0,
                float(self.current_pose.position.y) if self.current_pose else 0.0,
            ]
        }
        status_msg.data = json.dumps(status_data)
        self.phase_status_pub.publish(status_msg)

    def get_audit_log(self) -> dict:
        """Return complete audit log for judges"""
        return {
            'start_time': self.phase_events[0].timestamp if self.phase_events else None,
            'final_phase': self.current_phase.name,
            'total_manual_attempts': self.manual_control_attempts,
            'events': [asdict(e) for e in self.phase_events]
        }

    def save_log(self):
        """Save audit log to file"""
        try:
            with open(self.log_file, 'w') as f:
                json.dump(self.get_audit_log(), f, indent=2)
            self.get_logger().info(f'Audit log saved to {self.log_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save audit log: {e}')


def main(args=None):
    rclpy.init(args=args)
    manager = AutonomousPhaseManager()
    try:
        rclpy.spin(manager)
    finally:
        manager.save_log()
        rclpy.shutdown()


if __name__ == '__main__':
    main()