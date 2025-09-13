#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
from builtin_interfaces.msg import Duration

class ArmJointsTester(Node):
    def __init__(self):
        super().__init__('arm_joints_tester')

        # --- Only the 6 arm joints ---
        self.controller_joint_order = [
            "r_shoulder_swing_joint",
            "r_shoulder_lateral_joint",
            "r_elbow_joint",
            "l_shoulder_swing_joint",
            "l_shoulder_lateral_joint",
            "l_elbow_joint",
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
        self.timer = self.create_timer(2.0, self.start_test_once)
        self.test_started = False

    def joint_state_callback(self, msg: JointState):
        # Update dictionary of joint_name -> position
        self.joint_names = msg.name
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def start_test_once(self):
        if not self.test_started and len(self.joint_positions) > 0:
            self.test_started = True
            self.get_logger().info("Starting arm joint test sequence...")
            self.test_all_joints()

    def test_all_joints(self):
        # Example goal positions for testing (rad)
        goal_positions = {
            "r_shoulder_swing_joint": 1.474,
            "r_shoulder_lateral_joint":1.433,
            "r_elbow_joint": 1.054,
            "l_shoulder_swing_joint": 3.762,
            "l_shoulder_lateral_joint": 3.803,
            "l_elbow_joint": 4.182,
        }

        # Only loop through arm joints
        for joint in self.controller_joint_order:
            self.get_logger().info(f"Testing {joint} ...")
            self.move_single_joint(joint, goal_positions[joint])  # absolute goal
            time.sleep(2.0)  # wait for motion
            # Optionally move back to neutral
            #self.move_single_joint(joint, 0.0)
            #time.sleep(2.0)

        self.get_logger().info("? All arm joints tested successfully!")
        self.timer.cancel()  # stop the timer

    def move_single_joint(self, target_joint, goal_position):
        # Skip joints not in the controller list
        if target_joint not in self.controller_joint_order:
            self.get_logger().warn(f"Skipping {target_joint}: not in controller joint list")
            return

        # Get current positions to preserve other joints
        current_positions = [
            self.joint_positions.get(joint, 0.0) for joint in self.controller_joint_order
        ]

        # Set only the target joint to the goal position
        idx = self.controller_joint_order.index(target_joint)
        current_positions[idx] = goal_position

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = current_positions
        point.time_from_start = Duration(sec=2)

        # Build and publish trajectory
        traj = JointTrajectory()
        traj.joint_names = self.controller_joint_order
        traj.points.append(point)

        self.traj_pub.publish(traj)
        self.get_logger().info(f"Sent command: {target_joint} -> {goal_position:.3f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = ArmJointsTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
