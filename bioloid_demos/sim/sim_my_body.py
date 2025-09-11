#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FullBodyTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('full_body_trajectory_publisher')

        # Declare parameter (allows setting from launch file)
        #self.declare_parameter('use_sim_time', True)

        # Publisher (topic will be remapped in launch file if sim)
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            #'/gazebo_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.toggle = False

        # Define all 18 joint names (must match URDF / Gazebo controllers)
        self.joint_names = [
            'r_hip_twist_joint', 'r_hip_lateral_joint', 'r_hip_swing_joint',
            'r_knee_joint', 'r_ankle_swing_joint', 'r_ankle_lateral_joint',
            'l_hip_twist_joint', 'l_hip_lateral_joint', 'l_hip_swing_joint',
            'l_knee_joint', 'l_ankle_swing_joint', 'l_ankle_lateral_joint',
            'head_pan_joint', 'head_tilt_joint',
            'r_shoulder_joint', 'r_elbow_joint',
            'l_shoulder_joint', 'l_elbow_joint',
        ]

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

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.pose1 if self.toggle else self.pose2
        point.time_from_start.sec = 4  # complete in 4s

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

