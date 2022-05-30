cmake_minimum_required(VERSION 3.16)

project(RMCV VERSION 0.1.0)

# C++标准

set(CMAKE_CXX_STANDARD 20)

# 外部库（仅main.cpp所用）

find_package(OpenCV 4 REQUIRED)

# 模块

add_subdirectory(src/capture)
add_subdirectory(src/communicate)
add_subdirectory(src/config)
add_subdirectory(src/detect)
add_subdirectory(src/logging)
add_subdirectory(src/predict)
add_subdirectory(src/threading)
add_subdirectory(src/work_thread)

# RMCV

add_executable(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        src/main.cpp
        src/capture.cpp
        src/detect.cpp
        src/communicate.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE 
        capture
        communicate
        config
        detect
        logging
        predict
        threading
        work_thread
        
        ${OpenCV_LIBS}
)

# Tools

add_subdirectory(src/tool/calibrate)
