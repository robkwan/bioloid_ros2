from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('bioloid_description')
    
    # Read URDF file
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'typea.urdf')
    with open(urdf_file_path, 'r') as file:
        robot_description_content = file.read()
    
    # Robot description parameter
    robot_description = {'robot_description': robot_description_content}

    config = os.path.join(
        get_package_share_directory('dynamixel_bringup'),
        'config',
        'dxl_ros2_control.yaml'
        #'gazebo_ros2_control.yaml'
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, config],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"],
            output="screen"
        ),

    ])
