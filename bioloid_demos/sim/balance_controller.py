#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
import math
import tf2_ros
import tf2_geometry_msgs

class BalancingController(Node):
    def __init__(self):
        super().__init__('balancing_controller')

        # --- Parameters ---
        self.declare_parameter('kp', 5.0)  # Proportional gain
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.target_pitch = 0.0  # We want to stay upright

        # --- Subscribers and Publishers ---
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.cog_marker_publisher = self.create_publisher(Marker, '/cog_marker', 10)
        
        self.get_logger().info("Balancing Controller has started.")
        self.get_logger().info(f"Using Kp = {self.kp}")

    def imu_callback(self, msg: Imu):
        # 1. SENSE: Get orientation from IMU data
        orientation_q = msg.orientation
        # Convert quaternion to Euler angles
        (_, pitch, _) = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        # 2. PROCESS: Calculate control command using a P controller
        error = self.target_pitch - pitch
        control_output = self.kp * error  # This is the desired wheel velocity

        # 3. ACTUATE: Publish the command to ros2_control
        self.publish_command(control_output)

        # 4. VISUALIZE: Publish a marker for the CoG
        self.publish_cog_marker()

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_command(self, velocity_command):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [ " r_hip_twist_joint",           # ID 6
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
            "l_ankle_lateral_joint"       
        ]  # The joints to control

        point = JointTrajectoryPoint()
        point.velocities = [velocity_command]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds

        traj_msg.points.append(point)
        self.cmd_publisher.publish(traj_msg)
        
    def publish_cog_marker(self):
        """Publishes a marker to visualize the CoG in RViz."""
        marker = Marker()
        marker.header.frame_id = "base_link"  # Show CoG relative to the robot's base
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cog"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0  # Adjust this Z value to your robot's actual CoG height
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.cog_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = BalancingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
