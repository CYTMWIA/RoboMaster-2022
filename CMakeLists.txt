# Disable in-source builds to prevent source tree corruption.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "FATAL: 禁止在源文件目录 build")
endif()

cmake_minimum_required(VERSION 3.16)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} cmake)

project(REDO VERSION 0.1.0)

# 设置编译器
## C++20
set(CMAKE_CXX_STANDARD 20)
## 启用所有警告
add_compile_options(-Wall -Wextra -Wpedantic)
## 为 clangd 导出编译选项
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 全局启用的外部库

find_package(Boost 1.74 REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_libraries(${Boost_LIBRARIES})

find_package(OpenCV 4.5 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

find_package(fmt REQUIRED)
include_directories(SYSTEM ${fmt_INCLUDE_DIRS})
link_libraries(fmt::fmt)

find_package(yaml-cpp REQUIRED)
include_directories(SYSTEM ${YAML_CPP_INCLUDE_DIRS})
link_libraries(${YAML_CPP_LIBRARIES})

find_package(Ceres 2 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
link_libraries(${CERES_LIBRARIES})

# 模块

## 全局模块

add_subdirectory(src/nerv)
link_libraries(nerv)

## 普通模块

add_subdirectory(src/rm_capture)
add_subdirectory(src/rm_autoaim)
# add_subdirectory(src/rm_serial)

## 将普通模块也全局链接

link_libraries(rm_capture)
link_libraries(rm_autoaim)

# 可执行文件

add_executable(standard src/executables/standard.cpp)
add_executable(test src/executables/test.cpp)
# Tools

# add_executable(calibrate src/tool/calibrate/calibrate.cpp) # 来自OpenCV官方的标定工具
# add_executable(capture_camera src/tool/calibrate/capture_camera.cpp) # 相机捕获工具
# add_executable(usbreset src/tool/usbreset/usbreset.c) # 重置USB设备
