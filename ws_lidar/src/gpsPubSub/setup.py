from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gpsPubSub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
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
            'gps_publisher=gpsPubSub.GPS:main',
            'subscriber=gpsPubSub.subscribeGPS:main',
            'covariance=gpsPubSub.gps_covariance_calculation:main',
            'cov_comp=gpsPubSub.cov_comparison:main',
            'gps_odom=gpsPubSub.gps_odom:main',
            'position=gpsPubSub.pos_mon:main',
            'covariance_test=gpsPubSub.covariance_test:main',
            'imu_publisher=gpsPubSub.mpu_publisher:main'
        ],
    },
)
