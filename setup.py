from setuptools import find_packages, setup
import subprocess, os, platform
from glob import glob

package_name = 'gd_px4_control'

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
    maintainer='meard',
    maintainer_email='hansda.mahadev2@gmail.com',
    description='Python node for publishing hand gesture output to PX4 control ',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gd_px4_control = gd_px4_control.gd_px4_signal:main',
        ],
    },
)
