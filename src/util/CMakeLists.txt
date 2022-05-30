get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


add_library(${PROJECT_NAME})

target_sources( ${PROJECT_NAME}
    PRIVATE
        FpsCounter.cpp
)
