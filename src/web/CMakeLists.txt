get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


find_package(oatpp REQUIRED)

find_package(OpenCV 4 REQUIRED)

add_subdirectory(../config config_binary_dir)
add_subdirectory(../logging logging_binary_dir)
add_subdirectory(../threading threading_binary_dir)

add_subdirectory(../capture capture_binary_dir)

add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        web.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        config
        logging
        threading
        capture
        
        ${OpenCV_LIBS}
        oatpp::oatpp
)
