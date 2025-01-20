import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import datetime


def launch_setup(context, *args, **kwargs):
    interface_config = LaunchConfiguration('interface_config').perform(context)
    if interface_config.startswith('[') and interface_config.endswith(']'):
        interface_config = eval(interface_config)

    if not isinstance(interface_config, list):
        interface_config = [interface_config]

    node_to_start = [
        Node(
            package=[LaunchConfiguration('app_dir')],
            executable=[LaunchConfiguration(
                'interface'), '_', LaunchConfiguration('app')],
            # name=[LaunchConfiguration('interface'), '_', LaunchConfiguration('app')],
            output='screen',
            # prefix=[
            # 'valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=./valgrind_', datetime.datetime.now().isoformat('_', timespec='seconds'), '.log'],
            parameters=[
                LaunchConfiguration('hw_config'),
                LaunchConfiguration('control_config'),
                # {'feedback_mode': LaunchConfiguration('feedback_mode')}
            ]+interface_config
        ),

        # Uncomment the following lines if you want to include the ft_sensor_filter.launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([FindPackageShare('ohrc_perception'), '/launch/ft_sensor_filter.launch']),
        #     condition=IfCondition(LaunchConfiguration('use_ft_filter'))
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ]
    return node_to_start


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('app', default_value=''),
        DeclareLaunchArgument('robot', default_value='ur5e'),
        # vel or vel_trj or vel_pos
        DeclareLaunchArgument('controller', default_value='vel'),
        DeclareLaunchArgument('interface', default_value='marker'),
        # DeclareLaunchArgument(
        #     'feedback_mode', default_value='PositionFeedback'),
        DeclareLaunchArgument('use_ft_filter', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        DeclareLaunchArgument('app_dir', default_value=[
            'ohrc_', LaunchConfiguration('app')]),
        DeclareLaunchArgument('hw_config', default_value=[FindPackageShare(
            'ohrc_hw_config'), '/config/', LaunchConfiguration('robot'), '/', LaunchConfiguration('robot'), '_hw_config.yaml']),


        DeclareLaunchArgument('admittance_config', default_value=[
            FindPackageShare('ohrc_control'), '/config/admittance_config.yaml']),
        DeclareLaunchArgument('control_config', default_value=[FindPackageShare('ohrc_hw_config'), '/config/', LaunchConfiguration(
            'robot'), '/', LaunchConfiguration('robot'), '_control_config_', LaunchConfiguration('controller'), '.yaml']),
        DeclareLaunchArgument('interface_config', default_value=[LaunchConfiguration('app_dir'), '/config/', LaunchConfiguration('interface'), '_config.yaml']
                              ),

        DeclareLaunchArgument('rviz_config', default_value=[
            LaunchConfiguration('app_dir'), '/config/teleoperation.rviz']),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
