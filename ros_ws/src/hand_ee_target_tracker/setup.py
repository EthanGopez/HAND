from setuptools import find_packages, setup

package_name = 'hand_ee_target_tracker'

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
    maintainer='Abhiram',
    maintainer_email='abhiramjettyaj@gmail.com',
    description='Turns a slow end-effector goal into a bounded twist stream for MoveIt Servo.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ee_target_tracker = hand_ee_target_tracker.tracker_node:main',
            'send_goal = hand_ee_target_tracker.send_goal:main',
        ],
    },
)
