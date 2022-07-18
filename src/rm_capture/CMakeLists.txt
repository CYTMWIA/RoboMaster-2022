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
        /opt/HuarayTech/MVviewer/include
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        src/camera_node.cpp
        src/dahua_camera.cpp
        src/daheng_camera.cpp
        src/video_capture_node.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        /usr/lib/libgxiapi.so
        /opt/HuarayTech/MVviewer/lib/libMVSDK.so
)