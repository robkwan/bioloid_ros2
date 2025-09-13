#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SingleJointTest(Node):
    def __init__(self):
        super().__init__('single_joint_test')

        # Joint names MUST match the controller config exactly (order too!)
        self.joint_names = [
            "r_shoulder_swing_joint",
            "l_shoulder_swing_joint",
            "r_shoulder_lateral_joint",
            "l_shoulder_lateral_joint",
            "r_elbow_joint",
            "l_elbow_joint",
            "r_hip_twist_joint",
            "l_hip_twist_joint",   # Joint #8
            "r_hip_lateral_joint",   
            "l_hip_lateral_joint",
            "r_hip_swing_joint",
            "l_hip_swing_joint",
            "r_knee_joint",
            "l_knee_joint",
            "r_ankle_swing_joint",
            "l_ankle_swing_joint",
            "r_ankle_lateral_joint",
            "l_ankle_lateral_joint",
        ]

        # Publisher to trajectory controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.neutral = [0.0] * len(self.joint_names)
        self.test_index = 17 # index of joint

        # Start test
        self.get_logger().info("Starting single joint test (joint 8)")
        self.timer = self.create_timer(5.0, self.run_test)
        self.step = 0

    def run_test(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        positions = self.neutral.copy()

        if self.step == 0:
            positions[self.test_index] = 0.2 # -0.1   # move slightly +
            self.get_logger().info(f"Moving {self.joint_names[self.test_index]} -> {positions[self.test_index]} rad")
            point.time_from_start.sec = 2
        elif self.step == 1:
            positions[self.test_index] = -0.2 # 0.4  # move slightly -
            self.get_logger().info(f"Moving {self.joint_names[self.test_index]} -> {positions[self.test_index]} rad")
            point.time_from_start.sec = 2
        elif self.step == 2:
            positions[self.test_index] = 0.0    # back to neutral
            self.get_logger().info(f"Returning {self.joint_names[self.test_index]} -> 0.0 rad")
            point.time_from_start.sec = 2
        else:
            self.get_logger().info("Test complete. Holding neutral pose.")
            self.destroy_timer(self.timer)
            return

        point.positions = positions
        traj.points.append(point)
        self.publisher.publish(traj)
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = SingleJointTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
