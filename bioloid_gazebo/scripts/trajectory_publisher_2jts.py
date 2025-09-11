#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Publisher for JointTrajectory (optional: still publishes high-level message)
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Direct command publishers (relay to Gazebo)
        self.pub_r = self.create_publisher(Float64, '/r_shoulder_swing_joint/cmd_pos', 10)
        self.pub_l = self.create_publisher(Float64, '/l_shoulder_swing_joint/cmd_pos', 10)

        # Timer to publish trajectory periodically
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0

    def publish_trajectory(self):
        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['r_shoulder_swing_joint', 'l_shoulder_swing_joint']
        point = JointTrajectoryPoint()
        point.positions = [math.sin(self.t), math.cos(self.t)]
        point.time_from_start.sec = 1
        traj_msg.points.append(point)

        # Publish high-level trajectory
        self.traj_pub.publish(traj_msg)

        # Relay directly as joint commands
        r_cmd = Float64()
        l_cmd = Float64()
        r_cmd.data = point.positions[0]
        l_cmd.data = point.positions[1]
        self.pub_r.publish(r_cmd)
        self.pub_l.publish(l_cmd)

        self.get_logger().info(f"Publishing: r={r_cmd.data:.2f}, l={l_cmd.data:.2f}")

        # Increment time
        self.t += 0.05


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

