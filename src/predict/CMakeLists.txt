get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()


find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres 2 REQUIRED)

add_subdirectory(../logging logging_binary_dir)

add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        PnpSolver.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PUBLIC
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
    PRIVATE
        logging
)
