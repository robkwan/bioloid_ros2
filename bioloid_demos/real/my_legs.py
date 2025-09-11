#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class LegTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('leg_trajectory_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/leg_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.timer = self.create_timer(4.0, self.timer_callback)
        self.toggle = False

        # Define all 12 leg joints (must match YAML)
        self.joint_names = [
            'r_hip_twist_joint',
            'r_hip_lateral_joint',
            'r_hip_swing_joint',
            'r_knee_joint',
            'r_ankle_swing_joint',
            'r_ankle_lateral_joint',
            'l_hip_twist_joint',
            'l_hip_lateral_joint',
            'l_hip_swing_joint',
            'l_knee_joint',
            'l_ankle_swing_joint',
            'l_ankle_lateral_joint',
        ]

        # Define two alternating poses
        self.pose1 = [0.2, 0.1, 0.3, -0.5, 0.2, -0.1,
                      -0.2, -0.1, -0.3, 0.5, -0.2, 0.1]
        self.pose2 = [-0.2, -0.1, -0.3, 0.5, -0.2, 0.1,
                      0.2, 0.1, 0.3, -0.5, 0.2, -0.1]

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.pose1 if self.toggle else self.pose2
        point.time_from_start.sec = 3  # execute within 3s

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(f'Sent trajectory: {point.positions}')
        self.toggle = not self.toggle

def main(args=None):
    rclpy.init(args=args)
    node = LegTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

