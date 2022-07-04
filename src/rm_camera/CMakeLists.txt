cmake_minimum_required(VERSION 3.5)
project(rm_camera)

# 设置编译器
## C++14
set(CMAKE_CXX_STANDARD 14)
## 启用所有警告
add_compile_options(-Wall -Wextra -Wpedantic)
## 为 clangd 导出编译选项
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
## 大华
include_directories(SYSTEM /opt/HuarayTech/MVviewer/include)
link_libraries(/opt/HuarayTech/MVviewer/lib/libMVSDK.so)
## 大恒
link_libraries(/usr/lib/libgxiapi.so)

# Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/videocapture_node.cpp
  src/dahua_camera_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_camera::VideoCaptureNode
  EXECUTABLE videocapture_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_camera::DahuaCameraNode
  EXECUTABLE dahua_camera_node
)

# Install
ament_auto_package (
    INSTALL_TO_SHARE
    params
    launch
)
