from launch import LaunchDescription #Launch File..
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('hi_policy')
    map_package_path = get_package_share_directory('parking_world')
    # rviz_path = os.path.join(pkg_path, 'rviz', 'parking_map.rviz')  # 저장한 .rviz 파일명 사용
    map_visualizer_launch_file = os.path.join(map_package_path, 'launch', 'map_visualizer_launch.py')
    return LaunchDescription([
        Node(
            package = 'hi_policy',
            executable='multiple_agent_path',
            name = 'HLC_node',
            output='screen' #setup.py name

        ),
        Node(
            package = 'hi_policy',
            executable='vehicle_visualizer',
            name = 'vehicle',
            output='screen' #setup.py name

        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_visualizer_launch_file)
        )
    ])