#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import math
import time
import argparse
import os
import sys


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
        self.motion_steps = []

        # Initialize motion file path (will be set later)
        self.motion_file_path = None
        self.motion_name = ""
        self.compliance = []
        self.play_param = []

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

        self.get_logger().info(f"Sending trajectory with duration {duration:.2f}s...")
        self.get_logger().info(f"Joint positions (rad): {[f'{pos:.3f}' for pos in positions]}")

        # Print joint names + values
        self.get_logger().info("Joint details:")
        for i, (name, pos) in enumerate(zip(self.joint_names, positions)):
            self.get_logger().info(f"  ID {i:2d}: {name:25s} = {pos:7.3f} rad ({pos*180/math.pi:7.2f} deg)")

        # Send trajectory to action server
        self._action_client.wait_for_server()
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        if send_future.result().accepted:
            result_future = send_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            return result_future.result()
        else:
            self.get_logger().error("Goal was rejected!")
            return None

    def execute_motion_sequence_from_file(self, motion_file_path):
        """
        Execute a single motion file.
        """
        # Load the motion file
        self.set_motion_file(motion_file_path)
        
        # Print motion info
        motion_info = self.get_motion_info()
        self.get_logger().info(f"Executing motion: {motion_info['name']}")
        self.get_logger().info(f"  Steps: {motion_info['num_steps']}")
        self.get_logger().info(f"  File: {motion_info['file_path']}")
        
        # Execute the motion sequence
        self.execute_motion_sequence()

    def execute_motion_list(self, motion_files, base_path, delay_between_motions=1.0):
        """
        Execute a sequence of motion files with delays between them.
        """
        total_motions = len(motion_files)
        self.get_logger().info(f"Starting motion sequence with {total_motions} motion files...")
        
        for i, motion_filename in enumerate(motion_files):
            self.get_logger().info(f"=== Motion {i+1}/{total_motions}: {motion_filename} ===")
            
            # Construct full path
            motion_file_path = os.path.join(base_path, motion_filename)
            
            # Check if file exists
            if not os.path.isfile(motion_file_path):
                self.get_logger().error(f"Motion file not found: {motion_file_path}")
                self.get_logger().error("Skipping this motion...")
                continue
            
            # Execute the motion
            try:
                self.execute_motion_sequence_from_file(motion_file_path)
                self.get_logger().info(f"Completed motion: {motion_filename}")
            except Exception as e:
                self.get_logger().error(f"Error executing motion {motion_filename}: {e}")
                self.get_logger().error("Continuing with next motion...")
                continue
            
            # Delay between motions (except after the last one)
            if i < total_motions - 1 and delay_between_motions > 0:
                self.get_logger().info(f"Waiting {delay_between_motions:.1f}s before next motion...")
                time.sleep(delay_between_motions)
        
        self.get_logger().info("All motions in sequence completed!")

    def execute_motion_sequence(self):
        """
        Execute all motion steps.
        """
        if not self.motion_steps:
            self.get_logger().error("No motion steps loaded!")
            return
            
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
            result = self.send_trajectory(joint_positions, pose_duration if pose_duration > 0 else 2.0)

            # Wait for pose to complete (the action client already waits for completion)
            if result is None:
                self.get_logger().warn("Failed to execute trajectory step")

        self.get_logger().info("Motion sequence completed!")

    def set_motion_file(self, file_path):
        """
        Set a new motion file path and reload motion data.
        """
        self.motion_file_path = file_path
        self.load_motion_file()
    
    def get_motion_info(self):
        """
        Return dictionary with current motion information.
        """
        return {
           'name': self.motion_name,
           'compliance': self.compliance,
           'play_param': self.play_param,
           'num_steps': len(self.motion_steps),
           'file_path': self.motion_file_path
        }

    def load_motion_file(self, file_path=None):
        """
        Load motion data from .mtn file format.
        """
        if file_path is None:
            file_path = self.motion_file_path
    
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()
        
            # Reset motion data
            self.motion_name = ""
            self.compliance = []
            self.play_param = []
            self.motion_steps = []
        
            for line in lines:
                line = line.strip()
            
                # Skip empty lines and page markers
                if not line or line == "page_begin" or line == "page_end":
                    continue
            
                # Parse different line types
                if line.startswith("name="):
                    self.motion_name = line.split("=", 1)[1]
                
                elif line.startswith("compliance="):
                    compliance_str = line.split("=", 1)[1]
                    self.compliance = [int(x) for x in compliance_str.split()]
                
                elif line.startswith("play_param="):
                    play_param_str = line.split("=", 1)[1]
                    # Convert to appropriate types (mixed int/float)
                    params = play_param_str.split()
                    self.play_param = []
                    for param in params:
                        try:
                            # Try integer first
                            self.play_param.append(int(param))
                        except ValueError:
                            # If not integer, use float
                            self.play_param.append(float(param))
                
                elif line.startswith("step="):
                    step_str = line.split("=", 1)[1]
                    # Parse step data (mixed int/float)
                    step_values = step_str.split()
                    step_data = []
                    for value in step_values:
                        try:
                            # Try integer first
                            step_data.append(int(value))
                        except ValueError:
                            # If not integer, use float
                            step_data.append(float(value))
                    self.motion_steps.append(step_data)
        
            # Log loaded motion info
            self.get_logger().info(f"Loaded motion file: {file_path}")
            self.get_logger().info(f"Motion name: {self.motion_name}")
            self.get_logger().info(f"Compliance values: {self.compliance}")
            self.get_logger().info(f"Play parameters: {self.play_param}")
            self.get_logger().info(f"Number of motion steps: {len(self.motion_steps)}")
        
        except FileNotFoundError:
            self.get_logger().error(f"Motion file not found: {file_path}")
            raise FileNotFoundError(f"Motion file not found: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading motion file: {e}")
            raise Exception(f"Error loading motion file: {e}")

    def load_default_motion(self):
        """
        Load a basic default motion if no file is provided.
        This is a simple neutral pose.
        """
        self.motion_name = "default_neutral"
        self.compliance = [0] * len(self.motion_to_joint_map)
        self.play_param = [5000]  # Default play parameter
        
        # Create a single neutral step (all servos at 512 = 0 degrees)
        neutral_step = [0]  # dummy value
        neutral_step.extend([512] * 18)  # 18 joints at neutral position
        neutral_step.extend([0, 0])  # skip7, pause_duration, pose_duration
        neutral_step.extend([0.0, 2.0])  # pause=0s, hold=2s
        
        self.motion_steps = [neutral_step]
        self.get_logger().info("Loaded default neutral motion")


def read_motion_list_file(list_file_path):
    """
    Read a list file containing motion filenames.
    Returns a list of motion filenames.
    """
    motion_files = []
    
    try:
        with open(list_file_path, 'r') as file:
            lines = file.readlines()
        
        for line_num, line in enumerate(lines, 1):
            line = line.strip()
            
            # Skip empty lines and comments (lines starting with #)
            if not line or line.startswith('#'):
                continue
            
            # Check if line contains a valid motion filename
            if not line.endswith('.mtn'):
                print(f"Warning: Line {line_num} doesn't end with '.mtn': {line}")
                print(f"  Adding '.mtn' extension automatically")
                line = line + '.mtn'
            
            motion_files.append(line)
            print(f"  {len(motion_files):2d}: {line}")
    
    except FileNotFoundError:
        raise FileNotFoundError(f"List file not found: {list_file_path}")
    except Exception as e:
        raise Exception(f"Error reading list file: {e}")
    
    return motion_files


def validate_motion_files(motion_files, base_path):
    """
    Validate that all motion files in the list exist.
    Returns list of valid files and prints warnings for missing files.
    """
    valid_files = []
    missing_files = []
    
    for motion_file in motion_files:
        full_path = os.path.join(base_path, motion_file)
        if os.path.isfile(full_path):
            valid_files.append(motion_file)
        else:
            missing_files.append(motion_file)
    
    if missing_files:
        print(f"\nWarning: {len(missing_files)} motion file(s) not found:")
        for missing_file in missing_files:
            print(f"  - {missing_file}")
        
        if valid_files:
            print(f"\nWill proceed with {len(valid_files)} valid motion file(s)")
        else:
            raise FileNotFoundError("No valid motion files found!")
    
    return valid_files


def parse_arguments():
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser(description='ROS2 Motion Controller for Bioloid Robot (Action Client Version)')
    parser.add_argument('filename', 
                       help='Motion filename (e.g., bow.mtn) or list file (e.g., sequence.txt) containing multiple motion files')
    parser.add_argument('--path', 
                       default='/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions',
                       help='Base path to motion files directory (default: /home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions)')
    parser.add_argument('--list', 
                       action='store_true',
                       help='Treat input file as a list file containing multiple motion filenames')
    parser.add_argument('--delay', 
                       type=float, 
                       default=1.0,
                       help='Delay between motions in sequence (default: 1.0 seconds)')
    
    return parser.parse_args()


def main():
    # Parse command line arguments
    args = parse_arguments()
    
    # Determine if we're dealing with a single motion file or a list
    input_file_path = args.filename
    if not os.path.isabs(input_file_path):
        # If relative path, check in motion directory first, then current directory
        motion_dir_path = os.path.join(args.path, input_file_path)
        current_dir_path = input_file_path
        
        if os.path.isfile(motion_dir_path):
            input_file_path = motion_dir_path
        elif os.path.isfile(current_dir_path):
            input_file_path = current_dir_path
        else:
            print(f"Error: Input file not found: {input_file_path}")
            print(f"Searched in:")
            print(f"  Motion directory: {motion_dir_path}")
            print(f"  Current directory: {current_dir_path}")
            sys.exit(1)
    
    # Check if input file exists
    if not os.path.isfile(input_file_path):
        print(f"Error: Input file not found: {input_file_path}")
        sys.exit(1)
    
    # Determine mode: single motion file or motion list
    motion_files = []
    is_list_mode = args.list or not input_file_path.endswith('.mtn')
    
    if is_list_mode:
        print(f"List mode: Reading motion sequence from {input_file_path}")
        try:
            motion_files = read_motion_list_file(input_file_path)
            print(f"Found {len(motion_files)} motion files in list:")
            
            # Validate motion files
            motion_files = validate_motion_files(motion_files, args.path)
            print(f"Validated {len(motion_files)} motion files")
            
        except Exception as e:
            print(f"Error: {e}")
            sys.exit(1)
    else:
        print(f"Single motion mode: {input_file_path}")
        motion_files = [os.path.basename(input_file_path)]
        # Update args.path to be the directory of the single file
        args.path = os.path.dirname(input_file_path) or args.path
    
    rclpy.init()
    try:
        node = AllJointsTest()
        
        if is_list_mode and len(motion_files) > 1:
            # Execute motion sequence from list
            print(f"\nExecuting motion sequence:")
            print(f"  Motion files: {len(motion_files)}")
            print(f"  Base path: {args.path}")
            print(f"  Delay between motions: {args.delay}s\n")
            
            # Execute motion list
            node.execute_motion_list(motion_files, args.path, args.delay)
            
        else:
            # Single motion file mode
            motion_file_path = os.path.join(args.path, motion_files[0])
            print(f"Loading motion file: {motion_file_path}")
            
            # Load the specified motion file
            node.set_motion_file(motion_file_path)
            
            # Print motion info
            motion_info = node.get_motion_info()
            print(f"\nMotion Info:")
            print(f"  Name: {motion_info['name']}")
            print(f"  Steps: {motion_info['num_steps']}")
            print(f"  Compliance: {motion_info['compliance']}")
            print(f"  Play Params: {motion_info['play_param']}")
            print(f"  File: {motion_info['file_path']}\n")
            
            # Execute single motion
            node.execute_motion_sequence()
        
        node.get_logger().info("Motion execution completed!")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
