from setuptools import find_packages, setup

package_name = 'object_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/object_avoidance_launch.py']),
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
            'publisher=object_avoidance.object_avoidance_publisher:main',
            'subscriber=object_avoidance.object_avoidance_subscriber:main',
            'test=object_avoidance.test:main',
            'turntest=object_avoidance.turntest:main',
            'wall_follower=object_avoidance.wall_follower:main',
            'servo_turn=object_avoidance.servo_turn:main',
        ],
    },
)
