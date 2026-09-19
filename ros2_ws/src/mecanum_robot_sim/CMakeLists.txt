cmake_minimum_required(VERSION 3.8)
project(mecanum_robot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/chassis_motion_test.py
    scripts/clock_republisher.py
    scripts/collision_safety_node.py
    scripts/joint_state_republisher.py
    scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
