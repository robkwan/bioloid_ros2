#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class FullBodyTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('full_body_trajectory_publisher')

        # Publisher (topic will be remapped in launch file if sim)
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        self.timer = None
        self.toggle = False

        # Define all 18 joint names (must match URDF / Gazebo controllers)
        self.joint_names = [
            'r_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'r_elbow_joint',
            'l_shoulder_swing_joint', 'l_shoulder_lateral_joint', 'l_elbow_joint',
            'r_hip_twist_joint', 'r_hip_lateral_joint', 'r_hip_swing_joint',
            'r_knee_joint', 'r_ankle_swing_joint', 'r_ankle_lateral_joint',
            'l_hip_twist_joint', 'l_hip_lateral_joint', 'l_hip_swing_joint',
            'l_knee_joint', 'l_ankle_swing_joint', 'l_ankle_lateral_joint',
        ]

        # A stable initial pose. These are placeholder values.
        # Use joint_state_publisher_gui to find a real, stable pose.
        self.initial_pose = [0.0, 0.0, 0.0, -0.4, 0.0, 0.0,
                             0.0, 0.0, 0.0, -0.4, 0.0, 0.0,
                             0.0, 0.0,
                             0.0, 0.0,
                             0.0, 0.0]

        # Two simple poses for testing
        self.pose1 = [0.2, 0.1, 0.3, -0.5, 0.2, -0.1,
                      -0.2, -0.1, -0.3, 0.5, -0.2, 0.1,
                       0.3, -0.2,
                       0.4, 0.6,
                      -0.4, -0.6]

        self.pose2 = [-0.2, -0.1, -0.3, 0.5, -0.2, 0.1,
                       0.2, 0.1, 0.3, -0.5, 0.2, -0.1,
                      -0.3, 0.2,
                      -0.4, -0.6,
                       0.4, 0.6]
        
        # METHOD: Test full body movement after a delay.
        self.send_initial_pose()

    def send_initial_pose(self):
        self.get_logger().info('Sending initial standing pose. Please press "Start" in Gazebo to see the robot stand up.')
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.initial_pose
        point.time_from_start = Duration(seconds=5.0).to_msg()
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info('Initial pose sent. Starting timer for the main movement loop...')
        
        # Wait for the robot to settle before starting the main movement loop
        self.timer = self.create_timer(10.0, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.pose1 if self.toggle else self.pose2
        point.time_from_start = Duration(seconds=4.0).to_msg()

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(f'Sent trajectory: {point.positions}')
        self.toggle = not self.toggle

def main(args=None):
    rclpy.init(args=args)
    node = FullBodyTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

