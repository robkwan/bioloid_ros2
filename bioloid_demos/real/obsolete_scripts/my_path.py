#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import math
import time


class AllJointsTest(Node):
    def __init__(self):
        super().__init__('all_joints_test')

        # Action client for joint trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Service client to get joint names from controller parameters
        self._get_params_client = self.create_client(GetParameters,
                                                     '/joint_trajectory_controller/get_parameters')

        while not self._get_params_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /joint_trajectory_controller/get_parameters...')

        # Request the "joints" parameter
        req = GetParameters.Request()
        req.names = ['joints']
        future = self._get_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            joint_list = future.result().values[0].string_array_value
            self.joint_names = list(joint_list)
            self.get_logger().info(f"Discovered joints: {self.joint_names}")
        else:
            raise RuntimeError("Failed to get 'joints' parameter from joint_trajectory_controller!")

        # Define small test poses (~10 degrees ~= 0.17 rad)
        delta = 0.17
        self.neutral_pose = [0.0] * len(self.joint_names)
        self.pose1 = [delta] * len(self.joint_names)
        self.pose2 = [-delta] * len(self.joint_names)

    def send_trajectory(self, positions, duration=2.0):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        traj.points.append(point)
        goal_msg.trajectory = traj

        self.get_logger().info(f"Sending trajectory...")
        self._action_client.wait_for_server()
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)


def main():
    rclpy.init()
    node = AllJointsTest()

    # Sequence of moves
    node.send_trajectory(node.pose1, duration=3.0)
    time.sleep(4)

    node.send_trajectory(node.pose2, duration=3.0)
    time.sleep(4)

    node.send_trajectory(node.neutral_pose, duration=3.0)
    time.sleep(4)

    node.get_logger().info("Finished one full cycle. Holding neutral pose.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()

