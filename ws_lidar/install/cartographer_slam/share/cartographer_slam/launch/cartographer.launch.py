import launch
import launch_ros.actions
import os

def generate_launch_description():
    package_name = 'cartographer_slam'
    config_dir = os.path.join(
        launch_ros.substitutions.FindPackageShare(package_name).find(package_name),
        'config'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cartographer_ros',
            executable='cartographer_node',  # **Ensure executable is specified**
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer.lua'
            ],
            remappings=[
                ('scan', '/scan')
            ]
        ),
        launch_ros.actions.Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',  # **Ensure executable is specified**
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05
            }]
        )
    ])

