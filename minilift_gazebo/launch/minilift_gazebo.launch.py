import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "kavi_world.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'minilift.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "linorobot2"],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='linorobot2_gazebo',
            executable='command_timeout.py',
            name='command_timeout',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'publish_joints': 'false',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940

