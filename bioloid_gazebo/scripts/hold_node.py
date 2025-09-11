#!/usr/bin/env python3
# Simple hold-publisher node
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointHold(Node):
    def __init__(self):
        super().__init__('joint_hold')
        self.pub_r = self.create_publisher(Float64, '/r_shoulder_swing_joint/cmd_pos', 10)
        self.pub_l = self.create_publisher(Float64, '/l_shoulder_swing_joint/cmd_pos', 10)
        self.timer = self.create_timer(0.1, self.publish_hold)  # 10 Hz

    def publish_hold(self):
        msg_r = Float64()
        msg_r.data = 0.0
        msg_l = Float64()
        msg_l.data = 0.0
        self.pub_r.publish(msg_r)
        self.pub_l.publish(msg_l)

def main(args=None):
    rclpy.init(args=args)
    node = JointHold()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
