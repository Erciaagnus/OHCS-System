from setuptools import find_packages, setup

package_name = 'parking_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/map_visualizer_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henricus',
    maintainer_email='henricus0973@korea.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_visualizer = parking_world.map_visualizer:main',
            'rail_publisher = parking_world.rail_publisher:main',
        ],
    },
)
