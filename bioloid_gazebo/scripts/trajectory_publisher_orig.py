#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Define the joints you control
        self.joints = [
            'r_shoulder_swing_joint', 'r_shoulder_lateral_joint', 'r_elbow_joint',
            'l_shoulder_swing_joint', 'l_shoulder_lateral_joint', 'l_elbow_joint'
        ]
        
        # Publisher for JointTrajectory (optional: still publishes high-level message)
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        
        # Create direct command publishers for each joint dynamically
        self.joint_publishers = {}
        for joint in self.joints:
            topic_name = f'/{joint}/cmd_pos'
            self.joint_publishers[joint] = self.create_publisher(Float64, topic_name, 10)
        
        # Timer to publish trajectory periodically
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0
    
    def compute_joint_positions(self, t):
        """Compute desired positions for all 6 joints"""
        positions = []
        
        # Right arm joints
        positions.append(math.sin(t))                    # r_shoulder_swing_joint
        positions.append(0.3 * math.cos(t * 0.7))       # r_shoulder_lateral_joint  
        positions.append(-0.5 * math.sin(t * 1.2))      # r_elbow_joint
        
        # Left arm joints (mirrored/different pattern)
        positions.append(math.cos(t))                    # l_shoulder_swing_joint
        positions.append(-0.3 * math.cos(t * 0.7))      # l_shoulder_lateral_joint
        positions.append(-0.5 * math.sin(t * 1.2))      # l_elbow_joint
        
        return positions
    
    def publish_trajectory(self):
        # Compute joint positions
        positions = self.compute_joint_positions(self.t)
        
        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        traj_msg.points.append(point)
        
        # Publish high-level trajectory
        self.traj_pub.publish(traj_msg)
        
        # Publish direct joint commands
        for i, joint in enumerate(self.joints):
            cmd = Float64()
            cmd.data = positions[i]
            self.joint_publishers[joint].publish(cmd)
        
        # Log current positions
        pos_str = ", ".join([f"{joint.split('_')[0][0]}{joint.split('_')[1][0]}{joint.split('_')[2][0] if len(joint.split('_')) > 2 else ''}={pos:.2f}" 
                            for joint, pos in zip(self.joints, positions)])
        self.get_logger().info(f"Publishing: {pos_str}")
        
        # Increment time
        self.t += 0.05

def main():
    rclpy.init()
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
