#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch the map server, EKF node, and NavSat transform node."""
    # Get the package directory
    pkg_dir = get_package_share_directory('gpsPubSub')
    
    # Configuration file paths
    map_yaml_path = os.path.join(pkg_dir, 'config', 'ramp.yaml')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf_config.yaml')
    navsat_config_path = os.path.join(pkg_dir, 'config', 'navsat_config.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='nah'
    )
    # Start the map server
    start_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path},
                    {'use_sim_time': use_sim_time}]
    )
    
    # Start the NavSat transform node FIRST
    start_navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        respawn=True,  # Add respawn capability
        parameters=[navsat_config_path,
                    {'use_sim_time': use_sim_time}],
        remappings=[
        ('/imu', '/imu/data_filtered')],
    )
    
    # Start the EKF node AFTER NavSat
    start_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        respawn=True,  # Add respawn capability
        parameters=[ekf_config_path,
                    {'use_sim_time': use_sim_time}],
    )
    
    # Add static transform for GPS sensor position relative to base_link
    gps_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link']  # Adjust these values for your robot
    )
    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain_acc': 0.01,
            'do_bias_estimation': True,
            'bias_alpha': 0.01
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data'),
            ('/imu/data', '/imu/data_filtered')
        ]
    )
    
    map_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Start custom GPS to odometry converter
    gps_odom = Node(
        package='gpsPubSub',
        executable='gps_odom',
        name='gps_to_odom',
        output='screen',
    )
    
    # If the odom to base_link transform isn't being published by your gps_to_odom.py
    # (it should be, but just in case)
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    imu_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
    )

    gps_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
    )
    
    gps_publisher = Node(
        package='gpsPubSub',
        executable='gps_publisher',
        name='gps_publisher',
        output='screen'
    )
    imu_publisher = Node(
        package='gpsPubSub',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen'
    )


    # Create and return the launch description
    return LaunchDescription([
        map_to_odom,
        gps_to_base,
        odom_to_base,
        gps_publisher,
        imu_publisher,
        declare_use_sim_time,
        start_map_server,
        start_navsat,  # NavSat runs before EKF
        start_ekf,
        gps_transform,
        #gps_odom,
        imu_to_base,
        map_frame_publisher,
        imu_filter,
    ])
