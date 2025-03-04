from setuptools import find_packages, setup

package_name = 'teleop_control'

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
    maintainer='cewan',
    maintainer_email='chris.edward.wan@gmail.comp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		
                'teleop=teleop_control.teleop:main',
                'motor_control=teleop_control.motor_control:main',
        ],
    },
)
