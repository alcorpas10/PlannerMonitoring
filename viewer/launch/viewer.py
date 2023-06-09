from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution

import os

def get_viewer_node(context):
    config_file = LaunchConfiguration(
        'config_file').perform(context)

    execution_monitor_node = Node(
        package='viewer',
        namespace='planner',
        executable='viewer',
        name='viewer',
        parameters=[config_file],
        output='screen',
    )

    return [execution_monitor_node]

def generate_launch_description():
    launch_description = LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=''),
        OpaqueFunction(function=get_viewer_node)
    ])

    return launch_description
