#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.joints = [
            'r_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'r_elbow_joint',
            'l_shoulder_swing_joint', 'l_shoulder_lateral_joint', 'l_elbow_joint',
            'r_hip_twist_joint', 'r_hip_lateral_joint', 'r_hip_swing_joint',
            'r_knee_joint', 'r_ankle_swing_joint', 'r_ankle_lateral_joint',
            'l_hip_twist_joint', 'l_hip_lateral_joint', 'l_hip_swing_joint',
            'l_knee_joint', 'l_ankle_swing_joint', 'l_ankle_lateral_joint',
        ]

        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.joint_publishers = {
            j: self.create_publisher(Float64, f'/{j}/cmd_pos', 10)
            for j in self.joints
        }

        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0

    def compute_joint_positions(self, t):
        """
        Motion:
        - Right leg: kick forward repeatedly.
        - Left leg: stable/support leg.
        - Arms: both fold inward like hugging (symmetrical).
        """
        pos = []

        # === Arms hugging symmetrically ===
        hug = 0.5 * (math.sin(0.5*t) + 1.0)  # 0..1 wave for smooth closure
        # Right arm
        pos.append(-0.3 * hug)     # r_shoulder_swing_joint inward
        pos.append(0.4 * hug)      # r_shoulder_lateral_joint inward lift
        pos.append(0.7 * hug)      # r_elbow_joint bend
        # Left arm
        pos.append(0.3 * hug)      # l_shoulder_swing_joint inward
        pos.append(-0.4 * hug)     # l_shoulder_lateral_joint inward lift
        pos.append(0.7 * hug)      # l_elbow_joint bend

        # === Right leg (kicking) ===
        kick = 0.6 * math.sin(0.7*t)   # forward/backward hip swing
        knee = 0.7 * (math.sin(0.7*t + math.pi/4) + 1.0) / 2  # bend more when forward
        pos.append(0.0)           # r_hip_twist_joint
        pos.append(0.0)           # r_hip_lateral_joint
        pos.append(kick)          # r_hip_swing_joint
        pos.append(knee)          # r_knee_joint
        pos.append(-0.3*knee)     # r_ankle_swing_joint (foot adjusts)
        pos.append(0.0)           # r_ankle_lateral_joint

        # === Left leg (support leg) ===
        pos.append(0.0)           # l_hip_twist_joint
        pos.append(0.0)           # l_hip_lateral_joint
        pos.append(0.0)           # l_hip_swing_joint
        pos.append(0.1)           # l_knee_joint slight bend
        pos.append(0.0)           # l_ankle_swing_joint
        pos.append(0.0)           # l_ankle_lateral_joint

        return pos

    def publish_trajectory(self):
        positions = self.compute_joint_positions(self.t)

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        traj_msg.points.append(point)

        self.traj_pub.publish(traj_msg)
        for joint, val in zip(self.joints, positions):
            self.joint_publishers[joint].publish(Float64(data=val))

        self.get_logger().info(f"Kick+Hug positions: {[round(p,2) for p in positions]}")
        self.t += 0.05


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
