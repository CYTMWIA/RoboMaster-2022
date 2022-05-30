cmake_minimum_required(VERSION 3.16)

project(RMCV VERSION 0.1.0)

# C++标准

set(CMAKE_CXX_STANDARD 20)

# 全局启用的外部库

find_package(OpenCV 4 REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

find_package(fmt REQUIRED)
include_directories(SYSTEM ${fmt_INCLUDE_DIRS})
link_libraries(fmt::fmt)

# 全局include

include_directories(src)

# 模块

## 全局模块

add_subdirectory(src/logging)
link_libraries(logging)

add_subdirectory(src/threading)
link_libraries(threading)

## 普通模块

add_subdirectory(src/capture)
add_subdirectory(src/communicate)
add_subdirectory(src/config)
add_subdirectory(src/detect)
add_subdirectory(src/filter)
add_subdirectory(src/predict)
add_subdirectory(src/util)

## 将普通模块也全局链接（供线程、主程序、工具使用）

link_libraries(capture)
link_libraries(communicate)
link_libraries(config)
link_libraries(detect)
link_libraries(filter)
link_libraries(predict)
link_libraries(util)

# 线程

add_subdirectory(src/work_thread)
link_libraries(work_thread)

# 主程序 RMCV

add_executable(${PROJECT_NAME} src/main.cpp)

# Tools

add_executable(calibrate src/tool/calibrate/calibrate.cpp) # 来自OpenCV官方的标定工具
add_executable(capture_camera src/tool/calibrate/capture_camera.cpp) # 相机捕获工具
add_executable(usbreset src/tool/usbreset/usbreset.c) # 重置USB设备
