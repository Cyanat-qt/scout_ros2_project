cmake_minimum_required(VERSION 3.8)
project(navigation)

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
    scripts/centerline_param_tuner.py
    scripts/navigation_diagnostics.py
    scripts/orthogonal_centerline_controller.py
    scripts/recognition_line_commander.py
    scripts/recognition_route_commander.py
    scripts/route_sequence_runner.py
    scripts/yolo_camera_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
