from launch import LaunchDescription #Launch File..
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'parking_world',
            executable='map_visualizer',
            name = 'map_visualizer_node',
            output='screen' #setup.py name

        ),
        Node(
            package = 'parking_world',
            executable='rail_publisher',
            name = 'rail_publisher_node',
            output='screen' #setup.py name
        )
    ])