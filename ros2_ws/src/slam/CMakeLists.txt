cmake_minimum_required(VERSION 3.8)
project(slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_tf_broadcaster.py
    scripts/scan_frame_republisher.py
    scripts/scan_cone_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
