from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot', default_value='ur5e'),
        DeclareLaunchArgument('controller', default_value='vel'),
        # DeclareLaunchArgument(
        # 'feedback_mode', default_value='PositionFeedback'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('user_frame_viewpoint', default_value='back'),

        # Include the other launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(
                'ohrc_teleoperation'), '/launch/ohrc_teleoperation.launch.py']),
            launch_arguments={
                'interface': "['marker', 'joy_topic']",
                'robot': LaunchConfiguration('robot'),
                'controller': LaunchConfiguration('controller'),
                # 'feedback_mode': LaunchConfiguration('feedback_mode'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'user_frame_viewpoint': LaunchConfiguration('user_frame_viewpoint'),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(
                'spacemouse_ros2'), '/launch/spacemouse_publisher.launch.py']),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('device'), "' == 'spacenav'"])),
            launch_arguments={
                'joy_topic_name': 'cmd_joy',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(
                'keyboard'), '/launch/keyboard_to_joy.launch.py']),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('device'), "' == 'keyboard'"])),
            launch_arguments={
                'joy_topic_name': '/cmd_joy',
            }.items()
        )
    ])
