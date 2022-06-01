get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC 
        ${CMAKE_SOURCE_DIR}/3rdparty/
    PRIVATE
        /opt/HuarayTech/MVviewer/include
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        video_capture.cpp
        image_capture.cpp
        daheng_capture.cpp
        dahua_capture.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/3rdparty/daheng/libs/libgxiapi.so
        /opt/HuarayTech/MVviewer/lib/libMVSDK.so
)
