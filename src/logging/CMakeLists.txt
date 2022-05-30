get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


find_package(fmt REQUIRED)
include_directories(${fmt_INCLUDE_DIRS})


add_library(${PROJECT_NAME} INTERFACE)

target_include_directories( ${PROJECT_NAME}
    INTERFACE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries( ${PROJECT_NAME}
    INTERFACE
        fmt::fmt
)
