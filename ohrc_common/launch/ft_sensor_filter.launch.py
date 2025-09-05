import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import datetime


def launch_setup(context, *args, **kwargs):
    deadzone_force = eval(LaunchConfiguration('deadzone_force').perform(context))
    deadzone_torque = eval(LaunchConfiguration('deadzone_torque').perform(context))

    node_to_start = [
        Node(
            package='ohrc_tools',
            executable='ft_filter',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'ft_in': LaunchConfiguration('ft_topic_in')},
                {'ft_out': LaunchConfiguration('ft_topic_out')},
                {'sampling_freq': LaunchConfiguration('sampling_freq')},
                {'cutoff_freq': LaunchConfiguration('cutoff_freq')},
                {'lpf_order': LaunchConfiguration('lpf_order')},
                {'deadzone': {'force': {'lower': deadzone_force[0], 'upper': deadzone_force[1]},
                              'torque': {'lower': deadzone_torque[0], 'upper': deadzone_torque[1]}}
                }
            ]

        ),
    ]
    return node_to_start


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value='', description='namespace for the robot'),

        DeclareLaunchArgument('ft_topic_in', default_value='ft_sensor/raw', description='input topic name for the force-torque sensor'),
        DeclareLaunchArgument('ft_topic_out', default_value='ft_sensor/filtered', description='output topic name for the force-torque sensor'),

        DeclareLaunchArgument('sampling_freq', default_value='1000.0', description='sampling rate for applying filter'),
        DeclareLaunchArgument('cutoff_freq', default_value='200.0', description='cutoff frequency for the low-pass filter, which need to to be less than half of the sampling frequency'),

        DeclareLaunchArgument('lpf_order', default_value='2', description='order of the butterworth low-pass filter'),

        DeclareLaunchArgument('deadzone_force', default_value='[-0.2, 0.2]', description='deadzone for the force'),
        DeclareLaunchArgument('deadzone_torque', default_value='[-0.1, 0.1]', description='deadzone for the torque'),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
