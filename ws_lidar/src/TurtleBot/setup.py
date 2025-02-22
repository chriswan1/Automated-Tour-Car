from setuptools import find_packages, setup

package_name = 'TurtleBot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rrt_launch.py']),  # Install launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cewan',
    maintainer_email='cewan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rrt_node = TurtleBot.rrt_node:main",  # Becomes the executable 
            "Motion_Planner = TurtleBot.Motion_Planner:main",  # Becomes the executable 
            "PID_Controller = TurtleBot.PID_Controller:main",  # Becomes the executable 
            "map_subscriber = TurtleBot.map_reader_template:main",  # Becomes the executable 
        ],
    },
)
