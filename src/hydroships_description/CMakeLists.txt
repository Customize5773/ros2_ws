cmake_minimum_required(VERSION 3.8)
project(hydroships_description)

find_package(ament_cmake REQUIRED)

# Pasang folder aset supaya bisa ditemukan lewat package share directory.
install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

# Skrip bantu (estimasi massa/inertia dari geometri) — dipasang agar bisa
# dijalankan via `ros2 run` bila diinginkan; juga tetap jalan langsung dari src.
install(
  PROGRAMS scripts/estimate_mass_inertia.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
