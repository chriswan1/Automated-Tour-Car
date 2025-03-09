import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the publisher node
        Node(
            package='object_avoidance',
            executable='test_publisher',
            name='test_publisher',
            output='screen'
        ),

        # Start the subscriber node
        Node(
            package='object_avoidance',
            executable='test_subscriber',
            name='test_subscriber',
            output='screen'
        )
    ])