import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This is the new line that finds and installs your launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # Change this
    maintainer_email='your_email@email.com', # Change this
    description='ROS2 assignment for turtle autonomy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = turtle_autonomy.odometry_node:main',
            'autonomy_node = turtle_autonomy.autonomy_node:main',
        ],
    },
)
