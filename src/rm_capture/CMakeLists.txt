cmake_minimum_required(VERSION 3.5)
project(rm_capture)

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

# ROS2
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Others
find_package(OpenCV REQUIRED)
# Ours
find_package(rm_common REQUIRED)


add_library(rm_capture
  src/capture_node.cpp
  src/daheng_capture.cpp
  src/dahua_capture.cpp
)

ament_target_dependencies(rm_capture
  builtin_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  rm_common
)

target_include_directories(rm_capture
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/daheng/include>
  PRIVATE
    /opt/HuarayTech/MVviewer/include
)

target_link_libraries(rm_capture
  ${CMAKE_SOURCE_DIR}/3rdparty/daheng/lib/libgxiapi.so
  /opt/HuarayTech/MVviewer/lib/libMVSDK.so
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rm_capture
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  builtin_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  rm_common
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  rm_capture
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
