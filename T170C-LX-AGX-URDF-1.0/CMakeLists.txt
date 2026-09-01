cmake_minimum_required(VERSION 3.8)
project(t170c_lx_agx_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch textures
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
