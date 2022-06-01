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
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        pnp_solver.cpp
        aimer.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PUBLIC
        ${CERES_LIBRARIES}
)
