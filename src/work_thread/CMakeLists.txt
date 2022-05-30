get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

find_package(OpenCV 4 REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/src/capture capture_dir)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/communicate communicate_dir)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/config config_dir)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/detect detect_dir)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/logging logging_dir)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/predict predict_dir)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/threading threading_dir)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/util util_dir)

add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        CommunicateThread.cpp
        CaptureThread.cpp
        DetectThread.cpp
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
        util

        ${OpenCV_LIBS}
)