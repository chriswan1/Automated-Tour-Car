import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the publisher node
        Node(
            package='object_avoidance',
            executable='publisher',
            name='object_avoidance_publisher',
            output='screen'
        ),

        # Start the subscriber node
        Node(
            package='object_avoidance',
            executable='subscriber',
            name='object_avoidance_subscriber',
            output='screen'
        )
    ])