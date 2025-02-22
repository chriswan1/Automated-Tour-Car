import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start GPS Publisher
        Node(
            package='gpsPubSub',
            executable='publisher',
            name='gps_publisher',
            output='screen'
        ),

        # Start GPS Subscriber
        Node(
            package='gpsPubSub',
            executable='subscriber',
            name='gps_subscriber',
            output='screen'
        ),
    ])
