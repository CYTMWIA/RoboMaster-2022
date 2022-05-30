get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


# 串口库
file(GLOB_RECURSE serial_SOURCES ${CMAKE_SOURCE_DIR}/3rdparty/serial/src/**.cc)

add_subdirectory(../config config_binary_dir)
add_subdirectory(../logging logging_binary_dir)
add_subdirectory(../threading threading_binary_dir)


add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}
    PRIVATE
        serial/include
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        communicate.cpp
        ${serial_SOURCES}
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE 
        config
        logging
        threading
)
