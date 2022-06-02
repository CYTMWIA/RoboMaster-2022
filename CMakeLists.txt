# Disable in-source builds to prevent source tree corruption.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "FATAL: 禁止在源文件目录 build")
endif()

cmake_minimum_required(VERSION 3.16)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} cmake)

project(REDO VERSION 0.1.0)

# C++标准

set(CMAKE_CXX_STANDARD 20)

# 全局启用的外部库

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_libraries(${Boost_LIBRARIES})

find_package(OpenCV 4 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

find_package(fmt REQUIRED)
include_directories(SYSTEM ${fmt_INCLUDE_DIRS})
link_libraries(fmt::fmt)

# 模块

## 全局模块

add_subdirectory(src/rm_common)
link_libraries(rm_common)

## 普通模块

add_subdirectory(src/rm_capture)
add_subdirectory(src/rm_communicate)
add_subdirectory(src/rm_config)
add_subdirectory(src/rm_detect)
add_subdirectory(src/rm_predict)

## 将普通模块也全局链接（供线程、主程序、工具使用）

link_libraries(rm_capture)
link_libraries(rm_communicate)
link_libraries(rm_config)
link_libraries(rm_detect)
link_libraries(rm_predict)

# 线程

add_subdirectory(src/rm_node)
link_libraries(rm_node)

# 主程序

add_executable(REDO src/main.cpp)

# Tools

add_executable(calibrate src/tool/calibrate/calibrate.cpp) # 来自OpenCV官方的标定工具
add_executable(capture_camera src/tool/calibrate/capture_camera.cpp) # 相机捕获工具
add_executable(usbreset src/tool/usbreset/usbreset.c) # 重置USB设备
