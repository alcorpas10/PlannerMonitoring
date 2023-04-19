from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution

import os

def get_replanner_node(context):
    replanner_node = Node(
        package='replanning_manager',
        executable='replanning_manager',
        name='replanning_manager',
        output='screen',
    )

    return [replanner_node]

def generate_launch_description():
    launch_description = LaunchDescription([
        OpaqueFunction(function=get_replanner_node)
    ])

    return launch_description
