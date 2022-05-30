get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV 4 REQUIRED)

add_subdirectory(../logging logging_binary_dir)


add_library(${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        model.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        logging
)

if(NOT DEFINED MODEL_RUNNER)
    set(MODEL_RUNNER "openvino_sjtu")
endif()
message("当前 MODEL_RUNNER : " ${MODEL_RUNNER})

if(MODEL_RUNNER STREQUAL "tensorrt_sjtu")
    find_package(TensorRT REQUIRED)
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC USE_TENSORRT_SJTU
    )
    target_include_directories( ${PROJECT_NAME}
        PUBLIC ${TensorRT_INCLUDE_DIRS}
    )
    target_sources( ${PROJECT_NAME}
        PRIVATE
            tensorrt_sjtu/TRTLogger.cpp
            tensorrt_sjtu/TRTModule.cpp
    )
    target_link_libraries( ${PROJECT_NAME}
        PRIVATE
            ${TensorRT_LIBS}
            nvonnxparser
            cuda
    )
elseif(MODEL_RUNNER STREQUAL "openvino_sjtu")
    find_package(ngraph REQUIRED)
    find_package(InferenceEngine REQUIRED)
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC USE_OPENVINO_SJTU
    )
    target_sources( ${PROJECT_NAME}
        PRIVATE
            openvino_sjtu/vino_model.cpp
    )
    target_link_libraries( ${PROJECT_NAME}
        PRIVATE
            ${InferenceEngine_LIBRARIES}
            ${NGRAPH_LIBRARIES}
    )
else()
    message(FATAL_ERROR "未知 MODEL_RUNNER : " ${MODEL_RUNNER})
endif()
