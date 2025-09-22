#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench  # Use geometry_msgs for wrench
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class CoGStabilizer(Node):
    def __init__(self):
        super().__init__('cog_stabilizer')

        # Subscribe to IMU
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publisher for corrective force
        self.wrench_pub = self.create_publisher(Wrench, '/model/typea/link/base_link/wrench', 10)

        # PID gains (tune these!)
        self.kp_pitch = 400.0   # Torque to counter pitch (forward/back lean)
        self.kp_roll = 200.0    # Torque to counter roll (side lean)

        # Current orientation
        self.pitch = 0.0
        self.roll = 0.0
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("CoG stabilizer started. Waiting for IMU data...")

    def imu_callback(self, msg):
        # Extract quaternion
        q = msg.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        # Roll
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        self.roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            self.pitch = np.sign(sinp) * np.pi / 2
        else:
            self.pitch = np.arcsin(sinp)

        # Compute corrective torque to counter tilt
        torque_x = -self.kp_roll * self.roll        # Counter roll
        torque_y = -self.kp_pitch * self.pitch      # Counter pitch
        torque_z = 0.0                              # No yaw correction unless needed

        # Apply force to prevent falling forward/downward (optional)
        force_z = -10.0 * self.pitch  # Slight upward push if leaning forward

        # Send wrench to base link
        wrench = Wrench()
        wrench.force.x = 0.0
        wrench.force.y = 0.0
        wrench.force.z = force_z

        wrench.torque.x = torque_x
        wrench.torque.y = torque_y
        wrench.torque.z = torque_z

        self.wrench_pub.publish(wrench)

        # Log for debugging
        self.get_logger().info(
            f"Roll: {self.roll:.3f}, Pitch: {self.pitch:.3f}, "
            f"Torque: ({torque_x:.2f}, {torque_y:.2f}), Force: {force_z:.2f}N"
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'cog_estimate'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.01 - 0.05 * self.pitch  # Adjust CoG height slightly with tilt
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = CoGStabilizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
