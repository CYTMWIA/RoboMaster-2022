get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC 
        ${CMAKE_SOURCE_DIR}/3rdparty/
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        /opt/HuarayTech/MVviewer/include
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        src/video_capture.cpp
        src/image_capture.cpp
        src/daheng_capture.cpp
        src/dahua_capture.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        /usr/lib/libgxiapi.so
        /opt/HuarayTech/MVviewer/lib/libMVSDK.so
)
