#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
from builtin_interfaces.msg import Duration

class SingleJointTester(Node):
    def __init__(self):
        super().__init__('single_joint_tester')

        # --- Controller joint order (must match your controller YAML exactly!) ---
        self.controller_joint_order = [
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

        # Publisher to the joint trajectory controller
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store the latest joint states
        self.joint_positions = {}
 
        # Timer to start test after some seconds
        self.timer = self.create_timer(5.0, self.start_test_once)
        self.test_started = False

    def joint_state_callback(self, msg: JointState):
        # Update dictionary of joint_name -> position
        self.get_logger().info("In joint_state_callback...")
        self.joint_names = msg.name
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def start_test_once(self):
        if not self.test_started and len(self.joint_positions) > 0:
            self.test_started = True
            self.get_logger().info("Starting single-joint test sequence...")
            self.test_all_joints()

    def test_all_joints(self):
        for joint in self.joint_names:
            self.get_logger().info(f"Testing {joint} ...")
            self.move_single_joint(joint, target_offset=0.5)  # move by +0.5 rad
            time.sleep(2.0)  # wait for motion
            self.move_single_joint(joint, target_offset=0.0)  # back to original
            time.sleep(2.0)

        self.get_logger().info("? All joints tested successfully!")
        # Stop the timer (so start_test_once doesn't get called again)
        self.timer.cancel()
        
    def move_single_joint(self, target_joint, target_offset=0.5):
       # Skip joints not in the controller list
        if target_joint not in self.controller_joint_order:
            self.get_logger().warn(
                f"Skipping {target_joint}: not in controller joint list"
            )
            return

        # Create a point with all positions = 0
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.controller_joint_order)

        # Set only the target joint
        idx = self.controller_joint_order.index(target_joint)
        point.positions[idx] = target_offset
        point.time_from_start = Duration(sec=2)

        # Build and publish trajectory
        traj = JointTrajectory()
        traj.joint_names = self.controller_joint_order
        traj.points.append(point)

        self.traj_pub.publish(traj)
        self.get_logger().info(
            f"Sent command: {target_joint} ? {point.positions[idx]:.3f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SingleJointTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()  # shutdown from main thread only


if __name__ == '__main__':
    main()
