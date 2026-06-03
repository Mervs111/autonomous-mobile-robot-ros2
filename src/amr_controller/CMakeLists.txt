cmake_minimum_required(VERSION 3.8)
project(amr_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ============== C++ executable: stm32_bridge ==============
add_executable(stm32_bridge src/stm32_bridge.cpp)
ament_target_dependencies(stm32_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS
  stm32_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# ============== Python scripts: odometry_publisher ==============
install(PROGRAMS
  scripts/odometry_publisher.py
  scripts/imu_merger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============== Launch files ==============
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()
