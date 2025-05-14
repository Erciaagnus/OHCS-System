from launch import LaunchDescription #Launch File..
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('hi_policy')
    # rviz_path = os.path.join(pkg_path, 'rviz', 'parking_map.rviz')  # 저장한 .rviz 파일명 사용

    return LaunchDescription([
        Node(
            package = 'hi_policy',
            executable='global_path_planner',
            name = 'global_path_planner_node',
            output='screen' #setup.py name

        ),
    ])