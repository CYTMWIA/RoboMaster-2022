cmake_minimum_required(VERSION 3.5)
project(rm_interfaces)

# C99
set(CMAKE_C_STANDARD 99)
# C++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Angle3.msg
  msg/BoundingBox.msg
  msg/BoundingBoxesWithImageSize.msg
  msg/Rpy.msg
  msg/RpyStamped.msg
  msg/CmdToEc.msg

  DEPENDENCIES 
  std_msgs
  geometry_msgs
)

ament_package()
