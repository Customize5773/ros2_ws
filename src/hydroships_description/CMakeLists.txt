cmake_minimum_required(VERSION 3.8)
project(hydroships_description)

find_package(ament_cmake REQUIRED)

# Pasang folder aset supaya bisa ditemukan lewat package share directory.
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
