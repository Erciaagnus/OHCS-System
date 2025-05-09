cmake_minimum_required(VERSION 3.5)
project(parking_lot)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ament_cmake 및 의존성 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_dev QUIET)
find_package(gazebo_msgs QUIET)
find_package(gazebo_ros QUIET)

# include 디렉토리
include_directories(include)

# 노드가 있다면 여기서 빌드 (예시)
# add_executable(parking_node src/parking_node.cpp)
# ament_target_dependencies(parking_node rclcpp geometry_msgs nav_msgs std_msgs urdf tf2 tf2_ros gazebo_msgs gazebo_ros)

# install 설정 (예시)
# install(TARGETS
#   parking_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# launch, urdf, world 파일 복사 (필요시)
install(DIRECTORY
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 패키지 선언
ament_package()
