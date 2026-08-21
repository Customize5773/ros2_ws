cmake_minimum_required(VERSION 3.8)
project(hydroships_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config media models
  DESTINATION share/${PROJECT_NAME}
)

# Node spawner payload QR random (A/B/C/D). Dipasang sbg executable ROS 2
# ('ros2 run hydroships_gazebo payload_spawner') di lib/<pkg>.
install(
  PROGRAMS scripts/payload_spawner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME payload_spawner
)

# M5: validasi runtime payload terangkat & ketahanan grip (manual, bukan
# pytest). 'ros2 run hydroships_gazebo validate_grab_lift' setelah sim+FSM up.
install(
  PROGRAMS scripts/validate_grab_lift.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_grab_lift
)

# Ukur pose payload saat menggantung setelah AUTO_RELEASE (diagnostik hang
# accuracy). 'ros2 run hydroships_gazebo measure_hang_pose' bareng mission.
install(
  PROGRAMS scripts/measure_hang_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME measure_hang_pose
)

ament_package()
