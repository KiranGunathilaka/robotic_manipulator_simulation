from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'gp7_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kiran Gunathilaka',
    maintainer_email='kirangunathilaka@gmail.com',
    description='GP7 Robot Control Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place = gp7_robot.scripts.pick_place_controller:main',
            'test_kinematics = gp7_robot.scripts.test_kinematics:main',
        ],
    },
)
