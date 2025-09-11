#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self._client = ActionClient(self, FollowJointTrajectory, '/arm_joint_trajectory_controller/follow_joint_trajectory')
        
        # Define the joints you control
        self.joints = [
            'r_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'r_elbow_joint',
            'l_shoulder_swing_joint', 'l_shoulder_lateral_joint', 'l_elbow_joint'
        ]
        
        # Original "rest" positions
        self.original_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Positions for "arms up"
        self.up_positions = [1.0, 0.5, 0.0, 1.0, 0.5, 0.0]  # adjust to your robot
        # Positions for "arms open"
        self.open_positions = [1.0, 1.0, 0.5, 1.0, 1.0, 0.5]  # adjust to your robot

    def send_trajectory(self):
        self._client.wait_for_server()

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joints

        # Move arms up
        traj_msg.points.append(JointTrajectoryPoint(
            positions=self.up_positions,
            time_from_start=Duration(seconds=2).to_msg()
        ))

        # Open arms
        traj_msg.points.append(JointTrajectoryPoint(
            positions=self.open_positions,
            time_from_start=Duration(seconds=4).to_msg()
        ))

        # Return to original position
        traj_msg.points.append(JointTrajectoryPoint(
            positions=self.original_positions,
            time_from_start=Duration(seconds=6).to_msg()
        ))

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        # Send goal
        self._client.send_goal_async(goal_msg)
        self.get_logger().info('Trajectory sent! Arms will move up, open, then return.')

def main(args=None):
    rclpy.init(args=args)
    controller = ArmController()
    controller.send_trajectory()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
