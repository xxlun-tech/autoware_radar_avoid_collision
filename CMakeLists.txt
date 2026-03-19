cmake_minimum_required(VERSION 3.14)
project(autoware_radar_avoid_collision)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_avoid_collision.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::radar_avoid_collision::RadarAvoidCollision"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
