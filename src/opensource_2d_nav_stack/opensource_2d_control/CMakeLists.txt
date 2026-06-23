cmake_minimum_required(VERSION 3.8)
project(opensource_2d_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
