cmake_minimum_required(VERSION 3.5)
project(rm_common)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

add_library(rm_common
  src/rm_common.cpp
  src/image_msg.cpp
)

ament_target_dependencies(rm_common
  sensor_msgs
  OpenCV
  fmt
)

target_include_directories(rm_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rm_common
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  fmt
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  rm_common
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
