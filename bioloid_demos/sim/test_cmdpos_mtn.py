#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time
import threading
import argparse
import os
import sys


class AllJointsTest(Node):
    def __init__(self):
        super().__init__('all_joints_test')
 
        # Explicit mapping: step_data[1] -> r_shoulder_swing_joint, step_data[2] -> l_shoulder_swing_joint, etc.
        self.motion_to_joint_map = [
            "r_shoulder_swing_joint",      # ID 0
            "l_shoulder_swing_joint",      # ID 1
            "r_shoulder_lateral_joint",    # ID 2
            "l_shoulder_lateral_joint",    # ID 3
            "r_elbow_joint",               # ID 4
            "l_elbow_joint",               # ID 5
            "r_hip_twist_joint",           # ID 6
            "l_hip_twist_joint",           # ID 7
            "r_hip_lateral_joint",         # ID 8
            "l_hip_lateral_joint",         # ID 9
            "r_hip_swing_joint",           # ID 10
            "l_hip_swing_joint",           # ID 11
            "r_knee_joint",                # ID 12
            "l_knee_joint",                # ID 13
            "r_ankle_swing_joint",         # ID 14
            "l_ankle_swing_joint",         # ID 15
            "r_ankle_lateral_joint",       # ID 16
            "l_ankle_lateral_joint"        # ID 17
        ]

        # Define stable publishing order: legs first (ID 6-17), then arms (ID 0-5)
        self.stable_publishing_order = [
            # Legs first (ID 6-17) - critical for balance
            6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
            # Arms last (ID 0-5) - less critical for stability
            0, 1, 2, 3, 4, 5
        ]

        # Create trajectory publisher (optional, for debugging/monitoring)
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        
        # Create individual joint publishers
        self.joint_publishers = {
            j: self.create_publisher(Float64, f'/{j}/cmd_pos', 10)
            for j in self.motion_to_joint_map
        }

        # Subscribe to joint states for position feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Current joint positions and states
        self.current_positions = [0.0] * len(self.motion_to_joint_map)
        self.target_positions = [0.0] * len(self.motion_to_joint_map)
        self.motion_active = False
        self.joint_state_received = False
        self.actual_joint_positions = {}
        
        # Motion control parameters
        self.position_tolerance = 0.05  # radians
        self.max_wait_time = 10.0  # maximum time to wait for position (seconds)
        self.inter_group_delay = 0.05  # delay between leg and arm groups (seconds)
        
        # Timer for publishing joint commands
        self.timer = self.create_timer(0.01, self.publish_trajectory)  # 100Hz for smoother control

        # Use simulation time instead of wall time
        self.use_sim_time = True
        
        # Define the motion sequence
        # Format: [dummy, joint1...joint18, skip7, pause_duration, pose_duration]
        self.motion_steps = []
        
        # Initialize motion file path (will be set later)
        self.motion_file_path = None
        self.motion_name = ""
        self.compliance = []
        self.play_param = []

    def joint_state_callback(self, msg):
        """
        Callback to receive current joint positions from Gazebo.
        """
        self.joint_state_received = True
        
        # Update actual joint positions
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.motion_to_joint_map:
                self.actual_joint_positions[joint_name] = msg.position[i]

    def publish_trajectory_stable(self):
        """
        Publish joint positions in stable order: legs first (ID 6-17), then arms (ID 0-5).
        This helps maintain robot balance during motion transitions.
        """
        if not self.motion_active:
            return
            
        # Publish legs first (ID 6-17) - critical for balance and stability
        for joint_id in range(6, 18):  # IDs 6-17 (legs)
            if joint_id < len(self.motion_to_joint_map):
                joint_name = self.motion_to_joint_map[joint_id]
                msg = Float64()
                msg.data = self.target_positions[joint_id]
                self.joint_publishers[joint_name].publish(msg)
        
        # Small delay between leg and arm groups for better stability
        if self.inter_group_delay > 0:
            time.sleep(self.inter_group_delay)
        
        # Then publish arms (ID 0-5) - less critical for stability
        for joint_id in range(0, 6):  # IDs 0-5 (arms)
            if joint_id < len(self.motion_to_joint_map):
                joint_name = self.motion_to_joint_map[joint_id]
                msg = Float64()
                msg.data = self.target_positions[joint_id]
                self.joint_publishers[joint_name].publish(msg)

    def publish_trajectory(self):
        """
        Timer callback to continuously publish joint positions using stable ordering.
        """
        if self.motion_active:
            # Use stable publishing order instead of sequential order
            self.publish_trajectory_stable()
            
            # Optional: Also publish as trajectory message for monitoring
            self.publish_joint_trajectory()

    def publish_joint_trajectory(self):
        """
        Optional: Publish current target as JointTrajectory for monitoring/debugging.
        """
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.motion_to_joint_map
        
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.1 * 1e9)  # 100ms from now
        
        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)

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
        controller_joint_positions = [0.0] * len(self.motion_to_joint_map)
        for i, joint_name in enumerate(self.motion_to_joint_map):
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

    def set_joint_positions(self, positions):
        """
        Set target joint positions and activate motion with stability logging.
        """
        self.target_positions = positions.copy()
        self.motion_active = True
        
        self.get_logger().info("Setting joint positions (legs-first order for stability)...")
        self.get_logger().info(f"Joint positions (rad): {[f'{pos:.3f}' for pos in positions]}")

        # Print joint details in stability order
        self.get_logger().info("Joint details (publishing order: legs first, then arms):")
        
        # Show legs first (ID 6-17)
        self.get_logger().info("  LEGS (published first for stability):")
        for joint_id in range(6, 18):
            if joint_id < len(self.motion_to_joint_map):
                name = self.motion_to_joint_map[joint_id]
                pos = positions[joint_id]
                self.get_logger().info(f"    ID {joint_id:2d}: {name:25s} = {pos:7.3f} rad ({pos*180/math.pi:7.2f} deg)")
        
        # Show arms second (ID 0-5)
        self.get_logger().info("  ARMS (published second):")
        for joint_id in range(0, 6):
            if joint_id < len(self.motion_to_joint_map):
                name = self.motion_to_joint_map[joint_id]
                pos = positions[joint_id]
                self.get_logger().info(f"    ID {joint_id:2d}: {name:25s} = {pos:7.3f} rad ({pos*180/math.pi:7.2f} deg)")

    def wait_for_position_reached(self, target_positions, timeout=None):
        """
        Wait until all joints have reached their target positions or timeout.
        Uses actual joint feedback from Gazebo instead of fixed timing.
        """
        if timeout is None:
            timeout = self.max_wait_time
            
        start_time = self.get_clock().now()
        
        while True:
            # Check if we have joint state feedback
            if not self.joint_state_received:
                self.get_logger().warn("No joint state feedback received yet...")
                time.sleep(0.1)
                continue
            
            # Check if all joints are close to target
            all_reached = True
            legs_reached = True
            arms_reached = True
            
            for i, joint_name in enumerate(self.motion_to_joint_map):
                if joint_name in self.actual_joint_positions:
                    current_pos = self.actual_joint_positions[joint_name]
                    target_pos = target_positions[i]
                    error = abs(current_pos - target_pos)
                    
                    if error > self.position_tolerance:
                        all_reached = False
                        # Track which group hasn't reached target
                        if i >= 6:  # Legs (ID 6-17)
                            legs_reached = False
                        else:  # Arms (ID 0-5)
                            arms_reached = False
                else:
                    # Joint not found in feedback, assume not reached
                    all_reached = False
                    if i >= 6:
                        legs_reached = False
                    else:
                        arms_reached = False
            
            if all_reached:
                self.get_logger().info("All joints reached target positions")
                return True
            
            # Log progress for stability monitoring
            if not legs_reached and arms_reached:
                self.get_logger().debug("Arms ready, waiting for legs...")
            elif legs_reached and not arms_reached:
                self.get_logger().debug("Legs ready, waiting for arms...")
            
            # Check timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn(f"Timeout waiting for joints to reach position after {timeout}s")
                return False
            
            # Brief sleep and continue spinning
            time.sleep(0.05)
        
    def sim_time_sleep(self, duration):
        """
        Sleep for a duration using simulation time instead of wall time.
        This ensures proper timing even when Gazebo runs slower than real-time.
        """
        if duration <= 0:
            return
            
        start_time = self.get_clock().now()
        target_duration = rclpy.duration.Duration(seconds=duration)
        
        while (self.get_clock().now() - start_time) < target_duration:
            time.sleep(0.01)

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
        self.get_logger().info(f"  Stability mode: Legs-first publishing enabled")
        
        # Execute the motion sequence
        self.execute_motion_sequence()

    def execute_motion_list(self, motion_files, base_path, delay_between_motions=1.0):
        """
        Execute a sequence of motion files with delays between them.
        """
        total_motions = len(motion_files)
        self.get_logger().info(f"Starting motion sequence with {total_motions} motion files...")
        self.get_logger().info(f"Stability enhancement: Publishing legs (ID 6-17) before arms (ID 0-5)")
        
        # Wait for joint state feedback before starting
        self.get_logger().info("Waiting for joint state feedback...")
        while not self.joint_state_received:
            time.sleep(0.1)
        
        self.get_logger().info("Joint state feedback received, starting motion sequence...")
        
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
                self.sim_time_sleep(delay_between_motions)
        
        self.get_logger().info("All motions in sequence completed!")

    def execute_motion_sequence(self):
        """
        Execute all motion steps with Gazebo-safe timing and stability enhancements.
        """
        if not self.motion_steps:
            self.get_logger().error("No motion steps loaded!")
            return
            
        self.get_logger().info(f"Starting motion sequence with {len(self.motion_steps)} steps...")
        self.get_logger().info(f"Stability mode: Legs (ID 6-17) published before arms (ID 0-5)")
        
        # Wait for joint state feedback before starting (only if not already received)
        if not self.joint_state_received:
            self.get_logger().info("Waiting for joint state feedback...")
            while not self.joint_state_received:
                time.sleep(0.1)
            self.get_logger().info("Joint state feedback received, starting motion...")

        for i, step_data in enumerate(self.motion_steps):
            self.get_logger().info(f"Executing step {i+1}/{len(self.motion_steps)}")

            # Parse
            joint_positions, pause_duration, pose_duration = self.parse_motion_step(step_data)

            # Pause before movement (using simulation time)
            if pause_duration > 0:
                self.get_logger().info(f"Pausing for {pause_duration:.2f}s (sim time)...")
                self.sim_time_sleep(pause_duration)

            # Set joint positions (using stability-enhanced publishing)
            self.set_joint_positions(joint_positions)

            # Wait for joints to actually reach the target positions
            self.get_logger().info("Waiting for joints to reach target positions...")
            position_reached = self.wait_for_position_reached(joint_positions)
            
            if not position_reached:
                self.get_logger().warn("Some joints may not have reached target positions")

            # Hold pose for specified duration (using simulation time)
            if pose_duration > 0:
                self.get_logger().info(f"Holding pose for {pose_duration:.2f}s (sim time)...")
                self.sim_time_sleep(pose_duration)

        self.get_logger().info("Motion sequence completed!")

    def stop_motion(self):
        """
        Stop active motion publishing.
        """
        self.motion_active = False

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

    def set_inter_group_delay(self, delay):
        """
        Set the delay between leg and arm group publishing.
        Args:
            delay (float): Delay in seconds between leg and arm publishing (0 to disable)
        """
        self.inter_group_delay = delay
        self.get_logger().info(f"Inter-group delay set to {delay:.3f}s")


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
    parser = argparse.ArgumentParser(description='ROS2 Motion Controller for Bioloid Robot (Stability Enhanced)')
    parser.add_argument('filename', 
                       help='Motion filename (e.g., bow.mtn) or list file (e.g., sequence.txt) containing multiple motion files')
    parser.add_argument('--path', 
                       default='/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions',
                       help='Base path to motion files directory (default: /home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions)')
    parser.add_argument('--list', 
                       action='store_true',
                       help='Treat input file as a list file containing multiple motion filenames')
    parser.add_argument('--timeout', 
                       type=float, 
                       default=10.0,
                       help='Maximum time to wait for joint positions (default: 10.0 seconds)')
    parser.add_argument('--tolerance', 
                       type=float, 
                       default=0.05,
                       help='Position tolerance in radians (default: 0.05)')
    parser.add_argument('--delay', 
                       type=float, 
                       default=1.0,
                       help='Delay between motions in sequence (default: 1.0 seconds)')
    parser.add_argument('--group-delay', 
                       type=float, 
                       default=0.05,
                       help='Delay between leg and arm groups for stability (default: 0.05 seconds, 0 to disable)')
    
    return parser.parse_args()


def main():
    # Parse command line arguments
    args = parse_arguments()
    
    # Print stability enhancement info
    print("=== STABILITY ENHANCED MOTION CONTROLLER ===")
    print("Publishing order: Legs (ID 6-17) FIRST, then Arms (ID 0-5)")
    print(f"Inter-group delay: {args.group_delay:.3f}s")
    print("=" * 50)
    
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
        
        # Set motion control parameters from command line
        node.max_wait_time = args.timeout
        node.position_tolerance = args.tolerance
        node.set_inter_group_delay(args.group_delay)
        
        if is_list_mode and len(motion_files) > 1:
            # Execute motion sequence
            print(f"\nExecuting motion sequence:")
            print(f"  Motion files: {len(motion_files)}")
            print(f"  Base path: {args.path}")
            print(f"  Delay between motions: {args.delay}s")
            print(f"  Inter-group delay: {args.group_delay}s")
            print(f"  Timeout: {args.timeout}s")
            print(f"  Tolerance: {args.tolerance} rad\n")
            
            # Run motion sequence in a separate thread
            motion_thread = threading.Thread(
                target=node.execute_motion_list,
                args=(motion_files, args.path, args.delay)
            )
            motion_thread.start()
        else:
            # Single motion file mode
            motion_file_path = os.path.join(args.path, motion_files[0])
            print(f"Loading motion file: {motion_file_path}")
            
            node.set_motion_file(motion_file_path)
            
            # Print motion info
            motion_info = node.get_motion_info()
            print(f"\nMotion Info:")
            print(f"  Name: {motion_info['name']}")
            print(f"  Steps: {motion_info['num_steps']}")
            print(f"  Compliance: {motion_info['compliance']}")
            print(f"  Play Params: {motion_info['play_param']}")
            print(f"  File: {motion_info['file_path']}")
            print(f"  Timeout: {args.timeout}s")
            print(f"  Tolerance: {args.tolerance} rad")
            print(f"  Inter-group delay: {args.group_delay}s")
            print(f"  Stability: Legs-first publishing ENABLED\n")
            
            # Run single motion in a separate thread
            motion_thread = threading.Thread(target=node.execute_motion_sequence)
            motion_thread.start()
        
        # Spin the node to handle timer callbacks and joint state feedback
        rclpy.spin(node)
        
        motion_thread.join()
        node.get_logger().info("Motion execution completed!")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if 'node' in locals():
            node.stop_motion()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
