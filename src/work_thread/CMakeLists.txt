get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        src/communicate_thread.cpp
        src/capture_thread.cpp
        # src/detect_thread.cpp
        # src/strategy_thread.cpp
        src/ulm_xyz_kf.cpp
)