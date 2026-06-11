cmake_minimum_required(VERSION 3.8)
project(amr_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/goal_sender.py
  scripts/amr_auto_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
