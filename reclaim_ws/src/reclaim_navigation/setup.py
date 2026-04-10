import os
from glob import glob
from setuptools import setup

package_name = 'reclaim_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # Scripts
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RECLAIM Team',
    maintainer_email='reclaim@example.com',
    description='RECLAIM navigation stack: RPLIDAR, SLAM Toolbox, Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
