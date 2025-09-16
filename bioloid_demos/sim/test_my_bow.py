#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time
import threading


class AllJointsTest(Node):
    def __init__(self):
        super().__init__('all_joints_test')
 
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
        
        # Timer for publishing joint commands
        self.timer = self.create_timer(0.02, self.publish_trajectory)  # 50Hz for smoother control

        # Use simulation time instead of wall time
        self.use_sim_time = True
        
        # Define the motion sequence
        # Format: [dummy, joint1...joint18, skip7, pause_duration, pose_duration]
        self.motion_steps = []

    def joint_state_callback(self, msg):
        """
        Callback to receive current joint positions from Gazebo.
        """
        self.joint_state_received = True
        
        # Update actual joint positions
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.motion_to_joint_map:
                self.actual_joint_positions[joint_name] = msg.position[i]

    def publish_trajectory(self):
        """
        Timer callback to continuously publish joint positions.
        """
        if self.motion_active:
            # Publish to individual joint topics
            for i, joint_name in enumerate(self.motion_to_joint_map):
                msg = Float64()
                msg.data = self.target_positions[i]
                self.joint_publishers[joint_name].publish(msg)
            
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
        Set target joint positions and activate motion.
        """
        self.target_positions = positions.copy()
        self.motion_active = True
        
        self.get_logger().info("Setting joint positions...")
        self.get_logger().info(f"Joint positions (rad): {[f'{pos:.3f}' for pos in positions]}")

        # Print joint names + values
        self.get_logger().info("Joint details:")
        for i, (name, pos) in enumerate(zip(self.motion_to_joint_map, positions)):
            self.get_logger().info(f"  ID {i:2d}: {name:25s} = {pos:7.3f} rad ({pos*180/math.pi:7.2f} deg)")

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
            for i, joint_name in enumerate(self.motion_to_joint_map):
                if joint_name in self.actual_joint_positions:
                    current_pos = self.actual_joint_positions[joint_name]
                    target_pos = target_positions[i]
                    error = abs(current_pos - target_pos)
                    
                    if error > self.position_tolerance:
                        all_reached = False
                        break
                else:
                    # Joint not found in feedback, assume not reached
                    all_reached = False
                    break
            
            if all_reached:
                self.get_logger().info("All joints reached target positions")
                return True
            
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

    def execute_motion_sequence(self):
        """
        Execute all motion steps with Gazebo-safe timing.
        """
        self.get_logger().info(f"Starting motion sequence with {len(self.motion_steps)} steps...")
        
        # Wait for joint state feedback before starting
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

            # Set joint positions
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
            self.get_logger().error("Using default motion data...")
            # Fallback to hardcoded data if file not found
            self.load_default_motion()
        except Exception as e:
            self.get_logger().error(f"Error loading motion file: {e}")
            self.get_logger().error("Using default motion data...")
            self.load_default_motion()

def main():
    rclpy.init()
    try:
        node = AllJointsTest()
        
        # Optional: Load a different motion file
        node.set_motion_file("/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions/bow.mtn")
        
        # Print motion info
        motion_info = node.get_motion_info()
        print(f"\nMotion Info:")
        print(f"  Name: {motion_info['name']}")
        print(f"  Steps: {motion_info['num_steps']}")
        print(f"  Compliance: {motion_info['compliance']}")
        print(f"  Play Params: {motion_info['play_param']}")
        print(f"  File: {motion_info['file_path']}\n")
        
        # Run motion sequence in a separate thread to allow ROS2 spinning
        motion_thread = threading.Thread(target=node.execute_motion_sequence)
        motion_thread.start()
        
        # Spin the node to handle timer callbacks and joint state feedback
        rclpy.spin(node)
        
        motion_thread.join()
        node.get_logger().info("Holding final position...")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.stop_motion()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
