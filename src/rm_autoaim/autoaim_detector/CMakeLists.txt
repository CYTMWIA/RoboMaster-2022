cmake_minimum_required(VERSION 3.5)
project(autoaim_detector)

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

# Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/armor_detector_node.cpp
  src/armor_detector.cpp
  src/armor_icon_classifier.cpp
)

# Nodes
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_autoaim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

# Install
ament_auto_package (
  INSTALL_TO_SHARE
  assets
)
