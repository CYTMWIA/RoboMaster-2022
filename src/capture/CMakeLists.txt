get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


find_package(OpenCV 4 REQUIRED)

add_subdirectory(../logging logging_binary_dir)


add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC 
        ${CMAKE_SOURCE_DIR}/3rdparty/
    PRIVATE
        ${PROJECT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src/
        ${OpenCV_INCLUDE_DIRS}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        VideoCapture.cpp
        ImageCapture.cpp
        DahengCapture.cpp
        DahuaCapture.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        logging

        ${OpenCV_LIBS}
        ${CMAKE_SOURCE_DIR}/3rdparty/daheng/libs/libgxiapi.so
        /opt/HuarayTech/MVviewer/lib/libMVSDK.so
)
