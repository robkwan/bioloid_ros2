#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState
import math
import time
import argparse
import os
import sys
import threading

class AllJointsTest(Node):
    def __init__(self, motion_path=None):
        super().__init__('all_joints_test')

        # Publisher to joint_trajectory_controller - EXACTLY LIKE ORIGINAL
        self._traj_pub = self.create_publisher(JointTrajectory,
                                               '/joint_trajectory_controller/joint_trajectory',
                                               10)

        # Subscribe to standard joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to dynamic joint states (different message type)
        self.dynamic_joint_state_sub = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_state_callback,
            10
        )

        # Default joint names - EXACTLY LIKE ORIGINAL
        self.joint_names = [
            'r_shoulder_swing_joint','l_shoulder_swing_joint',
            'r_shoulder_lateral_joint','l_shoulder_lateral_joint',
            'r_elbow_joint','l_elbow_joint',
            'r_hip_twist_joint','l_hip_twist_joint',
            'r_hip_lateral_joint','l_hip_lateral_joint',
            'r_hip_swing_joint','l_hip_swing_joint',
            'r_knee_joint','l_knee_joint',
            'r_ankle_swing_joint','l_ankle_swing_joint',
            'r_ankle_lateral_joint','l_ankle_lateral_joint'
        ]

        # Mapping motion file indices to joint names - EXACTLY LIKE ORIGINAL
        self.motion_to_joint_map = list(self.joint_names)

        # NEW: Options for step-by-step control
        self.use_feedback = False
        self.position_tolerance = 0.05
        self.max_wait_time = 10.0
        self.inter_step_pause = 0.0  # Additional pause between steps
        
        # Joint state tracking
        self.joint_state_received = False
        self.actual_joint_positions = {}

        # Motion data - EXACTLY LIKE ORIGINAL
        self.motion_steps = []
        self.motion_name = ""
        self.play_param = []

        # Default motion path - EXACTLY LIKE ORIGINAL
        self.default_motion_path = motion_path or \
            '/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions'

    def joint_state_callback(self, msg):
        """Callback for standard JointState messages (/joint_states)"""
        if not self.joint_state_received:
            self.get_logger().info(f"Standard joint state feedback received ({len(msg.name)} joints)")
            
        self.joint_state_received = True
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_names:
                self.actual_joint_positions[joint_name] = msg.position[i]

    def dynamic_joint_state_callback(self, msg):
        """Callback for DynamicJointState messages (/dynamic_joint_states)"""
        if not self.joint_state_received:
            self.get_logger().info(f"Dynamic joint state feedback received ({len(msg.joint_names)} joints)")
            
        self.joint_state_received = True
        
        # DynamicJointState has a different structure
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.joint_names:
                # Extract position from interface_values
                interface_values = msg.interface_values[i]
                if 'position' in interface_values.interface_names:
                    pos_index = interface_values.interface_names.index('position')
                    self.actual_joint_positions[joint_name] = interface_values.values[pos_index]

    def load_motion_file(self, file_path):
        """Load a motion file (.mtn) - EXACTLY LIKE ORIGINAL"""
        self.motion_steps = []
        self.motion_name = ""
        self.play_param = []

        try:
            with open(file_path, 'r') as f:
                lines = f.readlines()
            for line in lines:
                line = line.strip()
                if not line or line.startswith('page_'):
                    continue
                if line.startswith('name='):
                    self.motion_name = line.split('=',1)[1]
                elif line.startswith('play_param='):
                    params = line.split('=',1)[1].split()
                    self.play_param = [float(p) if '.' in p else int(p) for p in params]
                elif line.startswith('step='):
                    step_data = []
                    for v in line.split('=',1)[1].split():
                        step_data.append(float(v) if '.' in v else int(v))
                    self.motion_steps.append(step_data)
            self.get_logger().info(f"Loaded motion '{self.motion_name}' with {len(self.motion_steps)} steps")
        except FileNotFoundError:
            self.get_logger().error(f"Motion file not found: {file_path}")
            raise
        except Exception as e:
            self.get_logger().error(f"Error loading motion file: {e}")
            raise Exception(f"Error loading motion file: {e}")

    def load_motion_list(self, file_path):
        """Read a .lst file and return list of .mtn file names"""
        motions = []
        try:
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    motions.append(line)
        except FileNotFoundError:
            print(f"Error: Motion list file not found: {file_path}")
            sys.exit(1)
        return motions

    def convert_to_radians(self, servo_value):
        """Convert Dynamixel-style value to radians - EXACTLY LIKE ORIGINAL"""
        degrees = ((servo_value - 512)/1023.0)*300.0
        return math.radians(degrees)

    def parse_motion_step(self, step_data):
        """Convert motion step to controller joint positions and timing - EXACTLY LIKE ORIGINAL"""
        motion_joint_values = step_data[1:19]  # skip dummy
        positions = [0.0]*len(self.joint_names)
        for i, name in enumerate(self.joint_names):
            if name in self.motion_to_joint_map:
                idx = self.motion_to_joint_map.index(name)
                positions[i] = self.convert_to_radians(motion_joint_values[idx])
        pause_duration = step_data[-2] if len(step_data)>=2 else 0
        pose_duration = step_data[-1] if len(step_data)>=1 else 2.0
        return positions, pause_duration, pose_duration

    def send_trajectory(self, positions, duration=2.0):
        """Publish trajectory point to the topic - EXACTLY LIKE ORIGINAL + time.sleep()"""
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration%1.0)*1e9)
        traj.points.append(point)

        self.get_logger().info(f"Publishing trajectory with duration {duration:.2f}s")
        for name,pos in zip(self.joint_names, positions):
            self.get_logger().info(f"{name:25s} = {pos:7.3f} rad ({math.degrees(pos):7.2f} deg)")

        # Publish and wait for duration - EXACTLY LIKE ORIGINAL
        self._traj_pub.publish(traj)
        time.sleep(duration)

    def wait_for_position_reached(self, target_positions, timeout=None):
        """Wait for joints to reach target positions (only if feedback is enabled)"""
        if not self.use_feedback:
            return True
            
        if timeout is None:
            timeout = self.max_wait_time
            
        # Wait a bit for joint feedback to arrive
        for _ in range(50):  # Wait up to 1 second
            if self.joint_state_received and len(self.actual_joint_positions) > 0:
                break
            time.sleep(0.1)
                
        if not self.joint_state_received or len(self.actual_joint_positions) == 0:
            self.get_logger().info(f"Condition 1: {self.joint_state_received} and Condition 2: {len(self.actual_joint_positions)}")
            self.get_logger().warn("No joint feedback available, skipping position check")
            return True
        
        matched_joints = [name for name in self.joint_names if name in self.actual_joint_positions]
        self.get_logger().info(f"Checking positions for {len(matched_joints)} joints (tolerance: {self.position_tolerance:.3f} rad)")
            
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            all_reached = True
            max_error = 0.0
            worst_joint = ""
            
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.actual_joint_positions:
                    current_pos = self.actual_joint_positions[joint_name]
                    target_pos = target_positions[i]
                    error = abs(current_pos - target_pos)
                    
                    if error > max_error:
                        max_error = error
                        worst_joint = joint_name
                    
                    if error > self.position_tolerance:
                        all_reached = False
                else:
                    all_reached = False
            
            if all_reached:
                self.get_logger().info(f"All joints reached target positions (max error: {max_error:.3f} rad)")
                return True
                
            time.sleep(0.1)
        
        self.get_logger().warn(f"Timeout waiting for positions (max error: {max_error:.3f} rad on {worst_joint})")
        return False

    def execute_motion_sequence(self):
        """Run all loaded motion steps - BASED ON ORIGINAL + OPTIONAL FEEDBACK"""
        if not self.motion_steps:
            self.get_logger().error("No motion steps loaded!")
            return
            
        self.get_logger().info(f"Executing motion sequence with {len(self.motion_steps)} steps...")
        
        if self.use_feedback:
            self.get_logger().info("Using position feedback mode")
        else:
            self.get_logger().info("Using original time-based mode")

        for i, step in enumerate(self.motion_steps):
            self.get_logger().info(f"Step {i+1}/{len(self.motion_steps)}")
            positions, pause, duration = self.parse_motion_step(step)
            
            # Pre-step pause from motion file
            if pause > 0:
                time.sleep(pause)
            
            # Send trajectory and wait (LIKE ORIGINAL)
            self.send_trajectory(positions, duration)
            
            # OPTIONAL: Also wait for position feedback
            if self.use_feedback:
                self.wait_for_position_reached(positions)
            
            # OPTIONAL: Additional inter-step pause
            if self.inter_step_pause > 0:
                self.get_logger().info(f"Inter-step pause: {self.inter_step_pause:.1f}s")
                time.sleep(self.inter_step_pause)
                
        self.get_logger().info("Motion sequence completed!")

    def set_feedback_mode(self, use_feedback):
        """Enable/disable position feedback"""
        self.use_feedback = use_feedback
        
    def set_inter_step_pause(self, pause):
        """Set additional pause between steps"""
        self.inter_step_pause = pause

def parse_arguments():
    parser = argparse.ArgumentParser(description="ROS2 Bioloid Motion Player")
    parser.add_argument('filename', help='Motion file (.mtn)')
    parser.add_argument('--path', default=None, help='Base path for motion files')
    parser.add_argument('--feedback', action='store_true',
                       help='Enable position feedback waiting')
    parser.add_argument('--step-pause', type=float, default=0.0,
                       help='Additional pause between steps (seconds)')
    parser.add_argument('--tolerance', type=float, default=0.05,
                       help='Position tolerance for feedback mode (radians)')
    return parser.parse_args()


def main():
    args = parse_arguments()
    base_path = args.path or '/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions'
    input_file = args.filename
    if not os.path.isabs(input_file):
        input_file = os.path.join(base_path, input_file)

    if not os.path.isfile(input_file):
        print(f"Error: File not found: {input_file}")
        sys.exit(1)

    rclpy.init()
    node = AllJointsTest(motion_path=base_path)
    try:
        # Configure options
        node.set_feedback_mode(args.feedback)
        node.set_inter_step_pause(args.step_pause)
        node.position_tolerance = args.tolerance

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # --- Check if .lst or .mtn ---
        if input_file.endswith(".lst"):
            motions = node.load_motion_list(input_file)
            print(f"Loaded motion list with {len(motions)} files")
            for i, motion_filename in enumerate(motions):
                motion_path = os.path.join(base_path, motion_filename)
                if not os.path.isfile(motion_path):
                    print(f"Warning: Motion file not found: {motion_path}")
                    continue
                node.load_motion_file(motion_path)
                print(f"\n[{i+1}/{len(motions)}] Motion: {node.motion_name}")
                node.execute_motion_sequence()
                time.sleep(1.0)  # optional delay between motions
        else:
            # Single .mtn file
            node.load_motion_file(input_file)
            print(f"\nMotion: {node.motion_name}")
            print(f"Steps: {len(node.motion_steps)}")
            node.execute_motion_sequence()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Proper cleanup sequence
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        
        if executor_thread is not None and executor_thread.is_alive():
            executor_thread.join(timeout=2.0)
        
        # Destroy the node before shutting down rclpy
        if node is not None:
            node.destroy_node()
        
        # Shutdown rclpy last
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
