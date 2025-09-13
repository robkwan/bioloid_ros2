#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
from builtin_interfaces.msg import Duration

class ArmJointTester(Node):
    def __init__(self):
        super().__init__('arm_joint_tester')

        # --- Controller joint order (full list, must match YAML) ---
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

        # Only arm joints to control
        self.arm_joints = [
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

        # Store latest positions
        self.joint_positions = {}
        self.timer = self.create_timer(2.0, self.start_test_once)
        self.test_started = False

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def start_test_once(self):
        if not self.test_started and len(self.joint_positions) >= len(self.controller_joint_order):
            self.test_started = True
            self.get_logger().info("Starting 6-arm-joint test...")
            self.test_arm_joints()
            self.timer.cancel()

    def test_arm_joints(self):
        # Example: move each arm joint to some absolute goal positions
        goal_positions_map = {
            "r_shoulder_swing_joint": 1.47,
            "r_shoulder_lateral_joint": 1.433,
            "r_elbow_joint": 1.054,
            "l_shoulder_swing_joint": 3.762,
            "l_shoulder_lateral_joint": 3.803,
            "l_elbow_joint": 4.182,
        }

        for joint, goal in goal_positions_map.items():
            self.get_logger().info(f"Moving {joint} to {goal:.3f} rad...")
            self.move_joint_to_goal(joint, goal)
            time.sleep(2.0)  # wait for movement

    def move_joint_to_goal(self, target_joint, goal_position, duration=2.0):
        # Make sure the joint is in controller list
        if target_joint not in self.controller_joint_order:
            self.get_logger().warn(f"Skipping {target_joint}: not in controller list")
            return

        # Build full joint position list (current positions for other joints)
        current_positions = [
            self.joint_positions.get(j, 0.0) for j in self.controller_joint_order
        ]

        # Update only the target joint
        idx = self.controller_joint_order.index(target_joint)
        current_positions[idx] = goal_position

        # Build trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.controller_joint_order

        point = JointTrajectoryPoint()
        point.positions = current_positions
        point.time_from_start = Duration(sec=int(duration))

        traj.points.append(point)

        self.traj_pub.publish(traj)
        self.get_logger().info(
            f"Published trajectory for {target_joint} -> {goal_position:.3f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArmJointTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
