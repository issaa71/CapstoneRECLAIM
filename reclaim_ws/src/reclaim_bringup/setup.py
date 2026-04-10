from setuptools import setup
import os
from glob import glob

package_name = 'reclaim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shady Siam',
    maintainer_email='shady@reclaim.dev',
    description='RECLAIM autonomous waste collection state machine',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waste_tracker = reclaim_bringup.waste_tracker:main',
            'waste_tracker_v2 = reclaim_bringup.waste_tracker_v2:main',
            'waste_tracker_trt = reclaim_bringup.waste_tracker_trt:main',
            'camera_bridge = reclaim_bringup.camera_bridge:main',
        ],
    },
)
