cmake_minimum_required(VERSION 3.5)
project(rm_bringup)

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

# 在整个 rm_bringup 内启用
find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_libraries(${Boost_LIBRARIES})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rm_common REQUIRED)
find_package(rm_autoaim REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/toml11
)

add_executable(legacy_control src/legacy_control.cpp)
ament_target_dependencies(legacy_control
  rclcpp
  rm_common
  rm_autoaim
)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  rm_common
  rm_autoaim
)

install(TARGETS 
  legacy_control
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
