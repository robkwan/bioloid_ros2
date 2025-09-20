#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():
    uses_gui_arg = DeclareLaunchArgument(
        'uses_gui', default_value='true', description='Whether to start Gazebo with GUI'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time in ROS nodes'
    )

    # Path to your package
    bioloid_prefix = get_package_prefix('bioloid_description')
    bioloid_share = get_package_share_directory('bioloid_description')

    # Extend GZ_SIM_RESOURCE_PATH
    set_resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.path.join(bioloid_share),    # where meshes/textures are
        ])
    )
    
    bioloid_gazebo_dir = FindPackageShare('bioloid_gazebo')

    world_file = PathJoinSubstitution([bioloid_gazebo_dir, 'worlds', 'empty3.sdf'])
    
    #sdf_file = PathJoinSubstitution([bioloid_gazebo_dir, 'worlds', 'part3_typea.sdf'])  # Adjust the path to your SDF file
  
    # Get absolute paths
    bioloid_description_dir = get_package_share_directory('bioloid_description')
    urdf_file = Path(bioloid_description_dir) / 'urdf' / 'part3_typea.urdf'

    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    # Initial joint positions as a dictionary
    initial_joint_states = {
        'r_shoulder_swing_joint': 0.0,
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
    gui_config_file = PathJoinSubstitution([bioloid_gazebo_dir, 'config', 'gui.config'])

  
    # Use DART physics (recommended / default)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            # NOTE: switch from Bullet -> DART --physics-engine gz-physics-bullet-plugin
            #'gz_args': [world_file, ' --physics-engine gz-physics-dartsim-plugin -v 2'],
            'gz_args': [world_file, ' --physics-engine gz-physics-dartsim-plugin -v 2 '],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    gz_ros2_control = Node(
        package="gz_ros2_control",
        executable="gz_ros2_control_system",
        name="gz_ros2_control",
        output="screen",
        parameters=[config_file],
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='typea',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/typea/robot_description', '-name', 'typea', 
                   '-z', '0.29', '-Y', '3.1416', 
                   '-joint_names', ','.join(joint_names),
                   '-joint_positions', ','.join(joint_positions)
                   ], #z = 0.2938
        #arguments=['-file', sdf_file, '-name', 'typea', '-z', '0.29', '-Y', '3.1416'],  # Adjust z and Y as needed
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Delay the spawn_robot node
    delayed_spawn = TimerAction(period=8.0, actions=[spawn_robot])
    
    # Minimal bridge: only /clock (let joint_state_broadcaster publish /joint_states)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   #'/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                   #'/r_shoulder_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_shoulder_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_shoulder_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_shoulder_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_elbow_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_elbow_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_hip_twist_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_hip_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_hip_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_ankle_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/r_ankle_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_hip_twist_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_hip_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_hip_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_knee_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_ankle_swing_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                   #'/l_ankle_lateral_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
                   ],
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
 
    # anti-gravity force node
    anti_gravity_node = Node(
        package='bioloid_gazebo',  # or wherever you put the node
        executable='anti_gravity.py',
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
        namespace='typea',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            config_file  # Load config directly
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        #namespace='typea',
        arguments=['joint_state_broadcaster', '-c', '/typea/controller_manager',
                   "--controller-manager-timeout", "120", "--switch-timeout", "100",
                   #'--param-file', '/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_gazebo/config/gazebo_ros2_control.yaml'
                   ],
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        #namespace='typea',
        arguments=[
            'joint_trajectory_controller',  '--controller-manager', '/typea/controller_manager',
            "--controller-manager-timeout", "120", "--switch-timeout", "100",
            '--param-file', '/home/robkwan/ros2_ws/src/bioloid_ros2/bioloid_gazebo/config/gazebo_ros2_control.yaml'
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
    
    delayed_controller_manager = TimerAction(
        period=3.0,  # Wait 2 seconds after robot_state_publisher starts
        actions=[controller_manager]
    )
       
    
    delayed_joint_state_broadcaster = TimerAction(
        period=12.0,  # Additional delay to ensure gz_ros2_control plugin is loaded
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_joint_trajectory_controller =TimerAction(
        period=15.0,
        actions=[joint_trajectory_controller_spawner]
    )
       
    return LaunchDescription([
        uses_gui_arg,
        use_sim_time_arg,
        set_resource_env,
        gazebo_launch,
        spawn_robot,
        #delayed_spawn,
        #bridge,
        #bridge_joint_states,
        #bridge_joint_commands,
        robot_state_publisher,
        #delayed_controller_manager,
        #controller_manager,
        #joint_state_broadcaster_spawner,
        #joint_trajectory_controller_spawner,
        delayed_joint_state_broadcaster,
        delayed_joint_trajectory_controller,
        #anti_gravity_node,
        #trajectory_node
        #hold_node
        #joint_setter,
        bridge,        
        # Wait 10 seconds for Gazebo and the robot_state_publisher to be fully ready
        #TimerAction(
        #    period=15.0,
        #    actions=[
        #        spawn_robot,
        #        #controller_manager,
        #        #gz_ros2_control,
        #        joint_state_broadcaster_spawner,
        #        joint_trajectory_controller_spawner,
        #    ]
        #)        
        
    ])

