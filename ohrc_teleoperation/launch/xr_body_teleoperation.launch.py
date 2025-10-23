from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot', default_value='ur5e'),
        DeclareLaunchArgument('controller', default_value='vel'),
        # DeclareLaunchArgument(
        # 'feedback_mode', default_value='HybridFeedback'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('user_frame_viewpoint', default_value='face'),

        # Include the other launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(
                'ohrc_teleoperation'), '/launch/ohrc_teleoperation.launch.py']),
            launch_arguments={
                'interface': 'xr_body',
                'robot': LaunchConfiguration('robot'),
                'controller': LaunchConfiguration('controller'),
                # 'feedback_mode': LaunchConfiguration('feedback_mode'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'user_frame_viewpoint': LaunchConfiguration('user_frame_viewpoint'),
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(
                'ros_tcp_endpoint'), '/launch/endpoint.py']),
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='temp_frame_broadcaster',
        #     arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0',
        #                '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'torso_link_tip'],
        # ),
    ])
