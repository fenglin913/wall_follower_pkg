#!/usr/bin/env python3
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wall_follower_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('wall_follower_pkg/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='feng',
    maintainer_email='826839151@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        f"wall_follower = wall_follower_pkg.wall_follower:main",

	],
    },
)
