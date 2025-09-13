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

        # Query controller to discover joint names
        self._get_params_client = self.create_client(
            GetParameters,
            '/joint_trajectory_controller/get_parameters'
        )

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

        self.current_positions = [0.0] * len(self.joint_names)
        
        # Explicit mapping: step_data[1] -> r_shoulder_swing_joint, step_data[2] -> l_shoulder_swing_joint, etc.
        self.motion_to_joint_map = [
            "r_shoulder_swing_joint",
            "l_shoulder_swing_joint",
            "r_shoulder_lateral_joint",
            "l_shoulder_lateral_joint",
            "r_elbow_joint",
            "l_elbow_joint",
            "r_hip_twist_joint",
            "l_hip_twist_joint",
            "r_hip_lateral_joint",
            "l_hip_lateral_joint",
            "r_hip_swing_joint",
            "l_hip_swing_joint",
            "r_knee_joint",
            "l_knee_joint",
            "r_ankle_swing_joint",
            "l_ankle_swing_joint",
            "r_ankle_lateral_joint",
            "l_ankle_lateral_joint"
        ]
        
        # Define the motion sequence
        # Format: [dummy, joint1...joint18, skip7, pause_duration, pose_duration]
        self.motion_steps = [
            [512, 335, 688, 279, 744, 462, 561, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 515, 512, 512, 512, 512, 512, 512, 512, 0.000, 0.296],
            [512, 288, 735, 280, 743, 206, 817, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 515, 512, 512, 512, 512, 512, 512, 512, 0.000, 0.496],

            #[512] + [300] + [512]*17 + [512]*7 + [0.0, 3.0],     # Joint 1 only
            #[512] + [512, 300] + [512]*16 + [512]*7 + [0.0, 3.0],  # Joint 2 only
            #[512] + [512]*2 + [300] + [512]*15 + [512]*7 + [0.0, 3.0],  # Joint 3 only
            #[512] + [512]*3 + [300] + [512]*14 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*4 + [300] + [512]*13 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*5 + [300] + [512]*12 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*6 + [300] + [512]*11 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*7 + [300] + [512]*10 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*8 + [300] + [512]*9  + [512]*7 + [0.0, 3.0],
            #[512] + [512]*9 + [300] + [512]*8  + [512]*7 + [0.0, 3.0],
            #[512] + [512]*10 + [300] + [512]*7 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*11 + [300] + [512]*6 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*12 + [300] + [512]*5 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*13 + [300] + [512]*4 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*14 + [300] + [512]*3 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*15 + [300] + [512]*2 + [512]*7 + [0.0, 3.0],
            #[512] + [512]*16 + [300] + [512]   + [512]*7 + [0.0, 3.0],
            #[512] + [512]*17 + [300]           + [512]*7 + [0.0, 3.0],  # Joint 18 only
        ]

    def convert_to_radians(self, servo_value: int) -> float:
        """
        Convert servo value to radians.
        0 = 0 degrees, 1023 = 300 degrees.
        """
        degrees = ((servo_value - 512)/ 1023.0) * 300.0
        return degrees * math.pi / 180.0


    def parse_motion_step(self, step_data):
        """
        Parse a motion step:
        - Skip index 0 (dummy)
        - Use indices [1:19] ? 18 joint values
        - Convert to radians
        - Return joint positions + timing
        """
        # Extract joint values
        motion_joint_servo_values = step_data[1:19]

        if len(motion_joint_servo_values) != len(self.motion_to_joint_map):
            raise ValueError(
                f"Motion step has {len(motion_joint_servo_values)} joints, "
                f"but mapping expects {len(self.motion_to_joint_map)}"
            )

        # Map explicitly to controller joints
        controller_joint_positions = [0.0] * len(self.joint_names)
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in self.motion_to_joint_map:
                idx = self.motion_to_joint_map.index(joint_name)
                controller_joint_positions[i] = self.convert_to_radians(motion_joint_servo_values[idx])
            else:
                # If not in mapping, keep at neutral (0 rad)
                controller_joint_positions[i] = 0.0

        # Timing (last 2 values)
        pause_duration = step_data[-2]
        pose_duration = step_data[-1]

        return controller_joint_positions, pause_duration, pose_duration

    def send_trajectory(self, positions, duration=2.0):
        """
        Send one trajectory point to the controller.
        """
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        traj.points.append(point)
        goal_msg.trajectory = traj

        self.get_logger().info(f"Would send trajectory with duration {duration:.2f}s...")
        self.get_logger().info(f"Joint positions (rad): {[f'{pos:.3f}' for pos in positions]}")

        # Print joint names + values
        self.get_logger().info("Joint details:")
        for i, (name, pos) in enumerate(zip(self.joint_names, positions)):
            self.get_logger().info(f"  ID {i:2d}: {name:25s} = {pos:7.3f} rad ({pos*180/math.pi:7.2f} deg)")

        # Commented out for safe testing:
        self._action_client.wait_for_server()
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        if send_future.result().accepted:
            result_future = send_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

    def execute_motion_sequence(self):
        """
        Execute all motion steps.
        """
        self.get_logger().info(f"Starting motion sequence with {len(self.motion_steps)} steps...")

        for i, step_data in enumerate(self.motion_steps):
            self.get_logger().info(f"Executing step {i+1}/{len(self.motion_steps)}")

            # Parse
            joint_positions, pause_duration, pose_duration = self.parse_motion_step(step_data)

            # Pause before movement
            if pause_duration > 0:
                self.get_logger().info(f"Pausing for {pause_duration:.2f}s...")
                time.sleep(pause_duration)

            # Send trajectory
            self.send_trajectory(joint_positions, pose_duration)

            # Wait for pose to complete
            if pose_duration > 0:
                self.get_logger().info(f"Waiting {pose_duration:.2f}s...")
                time.sleep(pose_duration)

        self.get_logger().info("Motion sequence completed!")


def main():
    rclpy.init()
    try:
        node = AllJointsTest()
        node.execute_motion_sequence()
        node.get_logger().info("Holding final position...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
