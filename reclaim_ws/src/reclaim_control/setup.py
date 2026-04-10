import os
from glob import glob
from setuptools import setup

package_name = 'reclaim_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shady Siam',
    maintainer_email='shady@todo.todo',
    description='Teensy bridge node for RECLAIM robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teensy_bridge = reclaim_control.teensy_bridge:main',
        ],
    },
)
