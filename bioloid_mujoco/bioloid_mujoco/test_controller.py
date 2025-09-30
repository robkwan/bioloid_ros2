#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(0.01, self.publish_commands)  # 100Hz
        self.t = 0.0
        
    def publish_commands(self):
        msg = Float64MultiArray()
        
        # Simple sinusoidal motion for demonstration
        # Adjust indices based on your joint order from /joint_states
        msg.data = [
            0.5 * math.sin(self.t),  # r_shoulder_swing
            0.0,                      # l_shoulder_swing
            0.3 * math.sin(self.t),  # r_shoulder_lateral
            0.0,                      # l_shoulder_lateral
            0.0,                      # r_elbow
            0.0,                      # l_elbow
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # hips and legs
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data[0]:.2f}')  # Add this line
        self.t += 0.02
    
def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
