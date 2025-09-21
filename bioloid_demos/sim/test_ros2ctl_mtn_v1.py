#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time
import argparse
import os
import sys

class AllJointsTest(Node):
    def __init__(self, motion_path=None):
        super().__init__('all_joints_test')

        # Publisher to joint_trajectory_controller
        self._traj_pub = self.create_publisher(JointTrajectory,
                                               '/joint_trajectory_controller/joint_trajectory',
                                               10)

        # Default joint names (discovered in your controller)
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

        # Mapping motion file indices to joint names
        self.motion_to_joint_map = list(self.joint_names)

        self.motion_steps = []
        self.motion_name = ""
        self.play_param = []

        # Default motion path (internal)
        self.default_motion_path = motion_path or \
            '/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions'

    def load_motion_file(self, file_path):
        """Load a motion file (.mtn)"""
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
            raise

    def convert_to_radians(self, servo_value):
        """Convert Dynamixel-style value to radians"""
        # 0-1023 -> -150 to +150 degrees -> radians
        degrees = ((servo_value - 512)/1023.0)*300.0
        return math.radians(degrees)

    def parse_motion_step(self, step_data):
        """Convert motion step to controller joint positions and timing"""
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
        """Publish trajectory point to the topic"""
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

        # Publish and wait for duration
        self._traj_pub.publish(traj)
        time.sleep(duration)

    def execute_motion_sequence(self):
        """Run all loaded motion steps"""
        if not self.motion_steps:
            self.get_logger().error("No motion steps loaded!")
            return
        self.get_logger().info(f"Executing motion sequence with {len(self.motion_steps)} steps...")
        for i, step in enumerate(self.motion_steps):
            self.get_logger().info(f"Step {i+1}/{len(self.motion_steps)}")
            positions, pause, duration = self.parse_motion_step(step)
            if pause>0:
                time.sleep(pause)
            self.send_trajectory(positions, duration)
        self.get_logger().info("Motion sequence completed!")

def parse_arguments():
    parser = argparse.ArgumentParser(description="ROS2 Bioloid Motion Player")
    parser.add_argument('filename', help='Motion file or list file')
    parser.add_argument('--path', default=None, help='Base path for motion files')
    parser.add_argument('--list', action='store_true', help='Use list file mode')
    parser.add_argument('--delay', type=float, default=1.0, help='Delay between motions')
    return parser.parse_args()

def main():
    args = parse_arguments()
    base_path = args.path or '/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_demos/motions'
    input_file = args.filename
    if not os.path.isabs(input_file):
        input_file = os.path.join(base_path, input_file)

    if not os.path.isfile(input_file):
        print(f"Error: Motion file not found: {input_file}")
        sys.exit(1)

    rclpy.init()
    node = AllJointsTest(motion_path=base_path)
    try:
        node.load_motion_file(input_file)
        # Wait a bit for controller ready
        time.sleep(1.0)
        node.execute_motion_sequence()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()
