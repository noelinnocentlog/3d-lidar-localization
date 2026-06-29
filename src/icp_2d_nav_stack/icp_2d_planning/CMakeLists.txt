cmake_minimum_required(VERSION 3.8)
project(icp_2d_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(include)

add_executable(hybrid_astar_planner
  src/reeds_shepp.cpp
  src/collision_checker.cpp
  src/heuristic.cpp
  src/hybrid_astar.cpp
  src/path_smoother.cpp
  src/planner_node.cpp
)

ament_target_dependencies(hybrid_astar_planner ${DEPS})

target_compile_features(hybrid_astar_planner PUBLIC cxx_std_17)

install(TARGETS hybrid_astar_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
