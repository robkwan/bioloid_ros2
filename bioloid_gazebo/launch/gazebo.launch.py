#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    uses_gui_arg = DeclareLaunchArgument(
        'uses_gui', default_value='true', description='Whether to start Gazebo with GUI'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time in ROS nodes'
    )

    bioloid_gazebo_dir = FindPackageShare('bioloid_gazebo')

    world_file = PathJoinSubstitution([bioloid_gazebo_dir, 'worlds', 'empty.sdf'])
    
    sdf_file = PathJoinSubstitution([bioloid_gazebo_dir, 'worlds', 'part2_typea.sdf'])  # Adjust the path to your SDF file
  
    # Get absolute paths
    bioloid_description_dir = get_package_share_directory('bioloid_description')
    urdf_file = Path(bioloid_description_dir) / 'urdf' / 'part2_typea.urdf'

    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    # Initial joint positions as a dictionary
    initial_joint_states = {
        'r_shoulder_swing_joint': 1.0,
        'r_shoulder_lateral_joint': 0.0,
        'r_elbow_joint': 0.0,
        'l_shoulder_swing_joint': 0.0, 
        'l_shoulder_lateral_joint': 0.0,
        'l_elbow_joint': 0.0,
        'r_hip_twist_joint': 0.0,
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
        # Add more joints as needed
    }
    
    # Convert to the expected format
    joint_names = []
    joint_positions = []
    for name, position in initial_joint_states.items():
        joint_names.append(name)
        joint_positions.append(str(position))
        
    robot_description = ParameterValue(urdf_content, value_type=str)

    # Configuration file path
    config_file = PathJoinSubstitution([bioloid_gazebo_dir, 'config', 'gazebo_ros2_control.yaml'])

  
    # Use DART physics (recommended / default)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            # NOTE: switch from Bullet -> DART --physics-engine gz-physics-bullet-plugin
            'gz_args': [world_file, ' --physics-engine gz-physics-dartsim-plugin -v 4 '],
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'typea', 
                   '-z', '0.29', '-Y', '3.1416', 
                   '-joint_names', ','.join(joint_names),
                   '-joint_positions', ','.join(joint_positions)
                   ], #z = 0.2938
        #arguments=['-file', sdf_file, '-name', 'typea', '-z', '0.29', '-Y', '3.1416'],  # Adjust z and Y as needed
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Minimal bridge: only /clock (let joint_state_broadcaster publish /joint_states)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                   '/r_shoulder_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_shoulder_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_shoulder_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_shoulder_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_elbow_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_elbow_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_hip_twist_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_hip_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_hip_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_ankle_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/r_ankle_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_hip_twist_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_hip_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_hip_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_ankle_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   '/l_ankle_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Hold joint positions node
    hold_node = Node(
        package='bioloid_gazebo',  # or wherever you put the node
        executable='hold_node.py',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ros_gz_bridge: joint_states (GZ -> ROS)
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ros_gz_bridge: joint commands (ROS -> GZ)
    bridge_joint_commands = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/r_shoulder_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/l_shoulder_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Trajectory publisher node
    trajectory_node = Node(
        package='bioloid_gazebo',
        executable='trajectory_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ros2_control controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            config_file  # Load config directly
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   ],
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            config_file,
            ],
    )
    
    # Reliable joint position setter after spawn
    joint_setter = TimerAction(
        period=4.0,  # Start after robot is spawned
        actions=[
            Node(
                package='bioloid_gazebo',
                executable='reliable_joint_setter.py',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )
    
    return LaunchDescription([
        uses_gui_arg,
        use_sim_time_arg,
        gazebo_launch,
        spawn_robot,
        bridge,
        #bridge_joint_states,
        #bridge_joint_commands,
        robot_state_publisher,
        #trajectory_node
        #hold_node
        #joint_setter,
    ])

