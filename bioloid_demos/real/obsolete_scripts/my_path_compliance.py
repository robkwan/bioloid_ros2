#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

# --- Joint names (must match URDF/controller) ---
JOINT_NAMES = [
    "r_shoulder_swing_joint",
    "r_shoulder_lateral_joint",
    "r_elbow_joint",
    "l_shoulder_swing_joint",
    "l_shoulder_lateral_joint",
    "l_elbow_joint",
    "r_hip_twist_joint",
    "r_hip_lateral_joint",
    "r_hip_swing_joint",
    "r_knee_joint",
    "r_ankle_swing_joint",
    "r_ankle_lateral_joint",
    "l_hip_twist_joint",
    "l_hip_lateral_joint",
    "l_hip_swing_joint",
    "l_knee_joint",
    "l_ankle_swing_joint",
    "l_ankle_lateral_joint",
]

# --- Example small-angle poses in radians (~5~10 deg) ---
POSE_1 = [0.1, 0.05, -0.05, 0.1, 0.05, -0.05,
          0.05, -0.05, 0.05, -0.05, 0.05, -0.05,
          0.05, -0.05, 0.05, -0.05, 0.05, -0.05]

POSE_2 = [-0.1, -0.05, 0.05, -0.1, -0.05, 0.05,
          -0.05, 0.05, -0.05, 0.05, -0.05, 0.05,
          -0.05, 0.05, -0.05, 0.05, -0.05, 0.05]

NEUTRAL = [0.0]*18

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('all_joints_compliance_test')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        time.sleep(2.0)  # wait before sending

        # Gradually move to Pose 1, Pose 2, then Neutral
        self.send_trajectory_smooth(POSE_1)
        self.send_trajectory_smooth(POSE_2)
        self.send_trajectory_smooth(NEUTRAL)
        self.get_logger().info("Finished trajectory. Holding Neutral pose.")

    def send_trajectory_smooth(self, target_pose, steps=10, step_delay=0.2):
        # Get current positions (for simplicity, assume neutral if first)
        current_pose = NEUTRAL
        for i in range(1, steps+1):
            intermediate_pose = [
                current + (target - current) * i / steps
                for current, target in zip(current_pose, target_pose)
            ]
            traj_msg = JointTrajectory()
            traj_msg.joint_names = JOINT_NAMES
            point = JointTrajectoryPoint()
            point.positions = intermediate_pose
            point.time_from_start.sec = 0
            traj_msg.points.append(point)
            self.publisher_.publish(traj_msg)
            self.get_logger().info(f"Sending intermediate pose step {i}/{steps}")
            time.sleep(step_delay)
        # Update current_pose for next call
        current_pose[:] = target_pose

def main():
    # Assuming AX IDs 1-18 for joints
    dxl_ids = list(range(1,19))

    rclpy.init()
    node = TrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
