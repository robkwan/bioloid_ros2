#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    uses_gui_arg = DeclareLaunchArgument(
        'uses_gui', default_value='true',
        description='Whether to start MuJoCo with GUI'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time in ROS nodes'
    )

    # Path to mujoco model (inside this package)
    bioloid_mujoco_share = get_package_share_directory('bioloid_mujoco')
    mujoco_model_file = Path(bioloid_mujoco_share) / 'urdf' / 'mujoco_typea_cor_stand.xml'
    controller_config = Path(bioloid_mujoco_share) / 'config' / 'mujoco_ros2_control.yaml'


    # MuJoCo simulator
    mujoco_sim = Node(
        package='mujoco_ros2',
        executable='mujoco_node',
        name='mujoco',
        output='screen',
        arguments=[str(mujoco_model_file)],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('uses_gui'),
            'joint_state_topic_name': 'joint_states',
            'joint_command_topic_name': 'joint_commands',
            'control_mode': 'POSITION',
            'simulation_frequency': 2000,
            'visualisation_frequency': 60,
            'camera_focal_point': [0.0, 0.0, 0.3],
            'camera_distance': 1.5,
            'camera_azimuth': 135.0,
            'camera_elevation': -20.0,
            'camera_orthographic': False
            #'ros2_control_config': str(controller_config) 
        }]
    )

    # Controller manager
    config_file = PathJoinSubstitution([bioloid_mujoco_share, 'config', 'mujoco_ros2_control.yaml'])
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    config_file],
        output='screen'
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        uses_gui_arg,
        use_sim_time_arg,
        mujoco_sim,
        #controller_manager,
        #joint_state_broadcaster_spawner,
        #joint_trajectory_controller_spawner,
    ])
