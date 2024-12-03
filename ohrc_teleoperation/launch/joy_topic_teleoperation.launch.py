from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot', default_value='ur5e'),
        DeclareLaunchArgument('controller', default_value='vel'),
        DeclareLaunchArgument(
            'feedback_mode', default_value='PositionFeedback'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('user_frame_viewpoint', default_value='back'),

        DeclareLaunchArgument('spacenav', default_value='true'),

        # Include the other launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(
                'ohrc_teleoperation'), '/launch/ohrc_teleoperation.launch.py']),
            launch_arguments={
                'interface': 'joy_topic',
                'robot': LaunchConfiguration('robot'),
                'controller': LaunchConfiguration('controller'),
                'feedback_mode': LaunchConfiguration('feedback_mode'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'user_frame_viewpoint': LaunchConfiguration('user_frame_viewpoint'),
            }.items()
        ),
        Node(
            package='spacenav',
            executable='spacenav_node',
            name='spacenav',
            condition=IfCondition(LaunchConfiguration('spacenav')),
            parameters=[
                    {
                        'zero_when_static': False,
                        'static_trans_deadband': 0.01,
                        'static_rot_deadband': 0.01,
                    }
            ],
            output='screen',
        ),
    ])
