from setuptools import find_packages, setup

package_name = 'gps_path_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tschew',
    maintainer_email='tschew@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_publisher=gps_path_tracking.GPS:main',
            'gps_tracking=gps_path_tracking.gps_tracking:main'
        ],
    },
)
