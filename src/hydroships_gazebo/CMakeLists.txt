cmake_minimum_required(VERSION 3.8)
project(hydroships_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config media
  DESTINATION share/${PROJECT_NAME}
)

# Node spawner payload QR random (A/B/C/D). Dipasang sbg executable ROS 2
# ('ros2 run hydroships_gazebo payload_spawner') di lib/<pkg>.
install(
  PROGRAMS scripts/payload_spawner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME payload_spawner
)

ament_package()
