get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


add_subdirectory(../logging logging_binary_dir)


add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME} 
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        MovingAverageFilter.cpp
        FpsCounter.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        logging
)
