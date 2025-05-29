from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hi_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hi_level_launch.py']),
        ( os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henricus',
    maintainer_email='henricus0973@korea.ac.kr',
    description='TODO: Package description',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_path_planner = hi_policy.global_path_planner:main',
            'ros_user_gui = hi_policy.ros_user_gui:main',
            'multiple_agent_path = hi_policy.multiple_agent_path:main',
            'vehicle_visualizer = hi_policy.vehicle_visualizer:main'
        ],
    },
)
