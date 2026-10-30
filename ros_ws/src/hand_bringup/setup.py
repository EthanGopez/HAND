import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hand_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhiram',
    maintainer_email='abhiramjettyaj@gmail.com',
    description='Launch files that connect HAND nodes with the xArm simulation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prepare_arm = hand_bringup.prepare_arm:main',
        ],
    },
)
