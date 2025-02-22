import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    map_yaml_path = "/home/cewan/Automated-Tour-Car/ws_lidar/engr_map.yaml"

    return LaunchDescription([
        # Start the PID Controller
        Node(
            package='TurtleBot',
            executable='PID_Controller',
            name='PID_Controller',
            output='screen'
        ),

        # Start the Motion Planner
        Node(
            package='TurtleBot',
            executable='Motion_Planner',
            name='Motion_Planner',
            output='screen'
        ),

        # Start the RRT node
        Node(
            package='TurtleBot',
            executable='rrt_node',
            name='rrt_node',
            output='screen'
        ),

        # Start the map server with updated YAML path
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml_path}],
            output='screen'
        ),

        # Lifecycle management for map server
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", "/map_server", "configure"],
            output="screen"
        ),
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                    output="screen"
                )
            ]
        ),

        # # Publish the target goal coordinates
        # ExecuteProcess(
        #     cmd=[
        #         "ros2", "topic", "pub", "-1", "/target_pose", "std_msgs/Float64MultiArray",
        #         "{data:[10.0,2.0]}"
        #     ],
        #     output="screen"
        # ),
    ])
