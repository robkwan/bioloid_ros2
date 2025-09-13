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
        
        # Define the motion sequence
        # Format: [joint1, joint2, ..., joint18, pause_duration, pose_duration]
        self.motion_steps = [
            [512, 335, 688, 279, 744, 462, 561, 353, 670, 508, 515, 347, 676, 282, 741, 617, 406, 508, 515, 512, 512, 512, 512, 512, 512, 512, 0.000, 0.296],
            [512, 288, 735, 280, 743, 206, 817, 353, 670, 508, 515, 347, 676, 282, 741, 617, 406, 508, 515, 512, 512, 512, 512, 512, 512, 512, 0.000, 0.496]#,
            #[512, 483, 540, 280, 743, 206, 817, 353, 670, 508, 515, 97, 926, 282, 741, 547, 476, 508, 515, 512, 512, 512, 512, 512, 512, 512, 0.496, 1.000],
            #[512, 288, 735, 280, 743, 206, 817, 353, 670, 508, 515, 347, 676, 282, 741, 617, 406, 508, 515, 512, 512, 512, 512, 512, 512, 512, 0.000, 1.000],
            #[512, 335, 688, 279, 744, 462, 561, 353, 670, 508, 515, 347, 676, 282, 741, 617, 406, 508, 515, 512, 512, 512, 512, 512, 512, 512, 0.000, 0.296],
            #[512, 235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 341, 682, 240, 783, 647, 376, 507, 516, 512, 512, 512, 512, 512, 512, 512, 0.000, 0.400]
        ]
    
    def convert_to_radians(self, servo_value):
        """
        Convert servo value to radians with proper offset:
        512 = 0 degrees = 0 radians (center position)
        0 = +150 degrees = +150 * pi/180 radians
        1023 = -150 degrees = -150 * pi/180 radians
        """
        # Calculate degrees: 512 is center (0 deg), range is +-150 deg
        #degrees = (512 - servo_value) * (150.0 / 512.0)
        #radians = degrees * math.pi / 180.0
        #return radians

        # Linear mapping: servo_value / 1023 * 300 degrees
        degrees = (servo_value / 1023.0) * 300.0
        radians = degrees * math.pi / 180.0
        return radians
     
    def parse_motion_step(self, step_data):
        """
        Parse a motion step and return joint positions in radians, pause duration, and pose duration
        Skip first value and last 7 values of 512 before timing parameters
        """
        # Skip the first value (index 0)
        # Extract joint values (indices 1 to 18, which gives us 18 joint values)
        joint_servo_values = step_data[1:19]  # Skip first, take next 18 values
        
        # Convert to radians
        joint_positions = [self.convert_to_radians(val) for val in joint_servo_values]
        
        # Adjust for actual joint count
        if len(self.joint_names) < len(joint_positions):
            joint_positions = joint_positions[:len(self.joint_names)]
        elif len(self.joint_names) > len(joint_positions):
            joint_positions.extend([0.0] * (len(self.joint_names) - len(joint_positions)))
        
        # Extract timing information (last 2 values)
        pause_duration = step_data[-2]  # Second to last value
        pose_duration = step_data[-1]   # Last value
        
        return joint_positions, pause_duration, pose_duration
    
    def send_trajectory(self, positions, duration=2.0):
        """
        Send a joint trajectory to the robot
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
        
        self.get_logger().info(f"Sending trajectory with duration {duration}s...")
        self.get_logger().info(f"Joint positions (rad): {[f'{pos:.3f}' for pos in positions]}")
        
        # Print joint names, IDs, and values
        self.get_logger().info("Joint details:")
        for i, (name, pos) in enumerate(zip(self.joint_names, positions)):
            self.get_logger().info(f"  ID {i:2d}: {name:10s} = {pos:7.3f} rad ({pos*180/math.pi:7.2f} deg)")        
        
        self._action_client.wait_for_server()
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        
        # Wait for the trajectory to complete
        if send_future.result().accepted:
            result_future = send_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
    
    def execute_motion_sequence(self):
        """
        Execute the entire motion sequence
        """
        self.get_logger().info(f"Starting motion sequence with {len(self.motion_steps)} steps...")
        
        for i, step_data in enumerate(self.motion_steps):
            self.get_logger().info(f"Executing step {i+1}/{len(self.motion_steps)}")
            
            # Parse the step
            joint_positions, pause_duration, pose_duration = self.parse_motion_step(step_data)
            
            # Apply pause before the movement if specified
            if pause_duration > 0:
                self.get_logger().info(f"Pausing for {pause_duration}s before movement...")
                time.sleep(pause_duration)
            
            # Send the trajectory
            self.send_trajectory(joint_positions, pose_duration)
            
            # Wait for the pose duration to complete (trajectory execution time)
            if pose_duration > 0:
                self.get_logger().info(f"Waiting {pose_duration}s for pose to complete...")
                time.sleep(pose_duration)
        
        self.get_logger().info("Motion sequence completed!")

def main():
    rclpy.init()
    
    try:
        node = AllJointsTest()
        
        # Execute the motion sequence
        node.execute_motion_sequence()
        
        # Hold the final position
        node.get_logger().info("Holding final position...")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
