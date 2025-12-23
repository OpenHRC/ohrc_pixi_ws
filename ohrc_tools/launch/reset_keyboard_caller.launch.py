from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='ohrc_tools',
            executable='reset_keyboard_caller',
            output='screen'
        ),

        Node(
            package='keyboard',
            executable='keyboard',
            output='screen'
        ),


    ])
