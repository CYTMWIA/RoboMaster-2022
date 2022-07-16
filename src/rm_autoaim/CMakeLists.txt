get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

find_package(Eigen3 REQUIRED)
find_package(Ceres 2 REQUIRED)

add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
    src/detector/detector_thread.cpp
    src/detector/power_rune_detector.cpp
    src/geometry/coordinate_converter.cpp
    src/geometry/pnp_solver.cpp
    src/predictor/predictor_thread.cpp
    src/predictor/power_rune_predictor.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PUBLIC
        ${CERES_LIBRARIES}
)