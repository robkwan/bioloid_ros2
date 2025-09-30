#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
import xacro

def generate_launch_description():
    uses_gui_arg = DeclareLaunchArgument(
        'uses_gui', default_value='true',
        description='Whether to start MuJoCo with GUI'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time in ROS nodes'
    )

    mujoco_ros2_control_urdf_path = os.path.join(
        get_package_share_directory('bioloid_mujoco'))

    xacro_file = os.path.join(mujoco_ros2_control_urdf_path,
                              'urdf',
                              'mujoco_typea_ros2ctl.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Path to mujoco model (inside this package)
    bioloid_mujoco_share = get_package_share_directory('bioloid_mujoco')
    mujoco_model_file = Path(bioloid_mujoco_share) / 'urdf' / 'mujoco_typea_ros2ctl.xml'
    controller_config_file = Path(bioloid_mujoco_share) / 'config' / 'mujoco_ros2_control.yaml'
    
    scene_file = os.path.join(bioloid_mujoco_share, 'urdf', 'scene.xml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': str(mujoco_model_file)},
            {'use_sim_time': False},
            {"scene_file": scene_file},
            #{'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'test_cart.xml')}
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        uses_gui_arg,
        #use_sim_time_arg,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher

    ])
