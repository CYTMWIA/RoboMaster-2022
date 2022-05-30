cmake_minimum_required(VERSION 3.16)

project(RMCV_Calibration VERSION 0.1.0)

# C++标准

set(CMAKE_CXX_STANDARD 20)

# 外部库

find_package(OpenCV 4 REQUIRED)

# 模块

add_subdirectory(../../capture capture_dir)
add_subdirectory(../../config config_dir)
add_subdirectory(../../logging logging_dir)
add_subdirectory(../../work_thread work_thread_dir)
add_subdirectory(../../threading threading_dir)

# Tools

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ../../
)
link_libraries(
    ${OpenCV_LIBS}
    capture
    config
    logging
    work_thread
    threading
)

set(EXECUTABLE_OUTPUT_PATH ../../../)

add_executable(calibrate calibrate.cpp)
add_executable(capture_camera capture_camera.cpp)
