from launch import LaunchDescription #Launch File..
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('parking_world')
    rviz_path = os.path.join(pkg_path, 'rviz', 'parking_map.rviz')  # 저장한 .rviz 파일명 사용

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
        ),
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        )
    ])