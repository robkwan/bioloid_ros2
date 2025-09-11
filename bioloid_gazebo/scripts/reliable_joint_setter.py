#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time

class ReliableJointSetter(Node):
    def __init__(self):
        super().__init__('reliable_joint_setter')
        
        # Define initial joint positions
        self.initial_positions = {
            'r_shoulder_swing_joint': 0.0,
            'r_shoulder_lateral_joint': 0.0,
            'r_elbow_joint': 0.0,
            'l_shoulder_swing_joint': 0.0,
            'l_shoulder_lateral_joint': 0.0,
            'l_elbow_joint': 0.0,
            'r_hip_twist_joint': 0.0,  # Your desired -1.0 position
            'r_hip_lateral_joint': 0.0,
            'r_hip_swing_joint': 0.0,
            'r_knee_joint': 0.0,
            'r_ankle_swing_joint': 0.0,
            'r_ankle_lateral_joint': 0.0,
            'l_hip_twist_joint': 0.0,
            'l_hip_lateral_joint': 0.0,
            'l_hip_swing_joint': 0.0,
            'l_knee_joint': 0.0,
            'l_ankle_swing_joint': 0.0,
            'l_ankle_lateral_joint': 0.0,
        }
        
        self.get_logger().info("Reliable joint setter starting...")
        
        # Wait for robot to be spawned, then set positions
        self.timer = self.create_timer(1.0, self.check_and_set_positions)
        self.attempt_count = 0
        self.max_attempts = 10
        self.positions_set = False
    
    def check_and_set_positions(self):
        if self.positions_set or self.attempt_count >= self.max_attempts:
            self.timer.cancel()
            return
        
        self.attempt_count += 1
        self.get_logger().info(f"Attempt {self.attempt_count} to set joint positions...")
        
        try:
            # First, check if the model exists
            list_cmd = ['gz', 'model', '--list']
            result = subprocess.run(list_cmd, capture_output=True, text=True, timeout=5)
            
            if 'typea' not in result.stdout:
                self.get_logger().info("Robot not yet spawned, waiting...")
                return
            
            self.get_logger().info("Robot found! Setting joint positions...")
            
            # Set each joint position using Gazebo service
            success_count = 0
            for joint_name, position in self.initial_positions.items():
                if self.set_joint_position(joint_name, position):
                    success_count += 1
            
            if success_count == len(self.initial_positions):
                self.get_logger().info(f"Successfully set all {success_count} joint positions!")
                self.positions_set = True
                self.timer.cancel()
                
                # Keep robot stable by continuing to publish positions
                self.stability_timer = self.create_timer(0.5, self.maintain_positions)
                self.stability_count = 0
            else:
                self.get_logger().warn(f"Only set {success_count}/{len(self.initial_positions)} positions")
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Timeout checking for robot")
        except Exception as e:
            self.get_logger().error(f"Error checking robot: {e}")
    
    def set_joint_position(self, joint_name, position):
        """Set individual joint position using Gazebo service"""
        try:
            # Method 1: Try joint position service
            cmd = [
                'gz', 'service', '-s', '/world/default/joint/position',
                '--reqtype', 'gz.msgs.JointPosition', 
                '--req', f'name: "typea::{joint_name}" position: [{position}]'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0:
                self.get_logger().info(f"? Set {joint_name} to {position}")
                return True
            else:
                # Method 2: Try alternative format
                cmd2 = [
                    'gz', 'topic', '-t', f'/model/typea/joint/{joint_name}/cmd_pos',
                    '-m', 'gz.msgs.Double',
                    '--data', f'data: {position}'
                ]
                
                result2 = subprocess.run(cmd2, capture_output=True, text=True, timeout=3)
                
                if result2.returncode == 0:
                    self.get_logger().info(f"? Set {joint_name} to {position} (method 2)")
                    return True
                else:
                    self.get_logger().warn(f"? Failed to set {joint_name}: {result.stderr}")
                    return False
                    
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Timeout setting {joint_name}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error setting {joint_name}: {e}")
            return False
    
    def maintain_positions(self):
        """Keep publishing positions to maintain stability"""
        if self.stability_count < 20:  # Run for 10 seconds
            for joint_name, position in self.initial_positions.items():
                cmd = [
                    'gz', 'topic', '-t', f'/model/typea/joint/{joint_name}/cmd_pos',
                    '-m', 'gz.msgs.Double', 
                    '--data', f'data: {position}'
                ]
                subprocess.run(cmd, capture_output=True, timeout=1)
            
            self.stability_count += 1
        else:
            self.stability_timer.cancel()
            self.get_logger().info("Position maintenance complete")

def main(args=None):
    rclpy.init(args=args)
    node = ReliableJointSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
