cmake_minimum_required(VERSION 3.8)
project(amr_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/goal_sender.py
  scripts/amr_auto_patrol.py
  scripts/amr_loop_patrol.py
  scripts/log_localization.py
  scripts/reset_odom.sh
  scripts/demo_drive_forward.sh
  scripts/run_amr_demo.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
