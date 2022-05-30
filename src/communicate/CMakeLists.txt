get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


# 串口库
file(GLOB_RECURSE serial_SOURCES ${CMAKE_SOURCE_DIR}/3rdparty/serial/src/**.cc)

find_package(Boost)


add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PRIVATE
        ${Boost_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/3rdparty/serial/include
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        Serial.cpp
        Cmd.c
        Vofa.cpp
        ${serial_SOURCES}
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
)
