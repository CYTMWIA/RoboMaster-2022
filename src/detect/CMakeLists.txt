get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_library(${PROJECT_NAME})

target_sources( ${PROJECT_NAME}
    PRIVATE
        cv_util.cpp
        HammingMatch.cpp
        MatchsClassifier.cpp
        CvArmorDetector.cpp
        NnArmorDetector.cpp
        BoundingBox.cpp
)

if(NOT DEFINED MODEL_RUNNER)
    set(MODEL_RUNNER "none")
endif()
message("当前 MODEL_RUNNER : " ${MODEL_RUNNER})

if(MODEL_RUNNER MATCHES "none")
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC MODEL_RUNNER_NONE
    )
elseif(MODEL_RUNNER MATCHES "tensorrt.*$")
    find_package(TensorRT REQUIRED)
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC MODEL_RUNNER_TENSORRT_SJTU
    )
    target_include_directories( ${PROJECT_NAME}
        SYSTEM PUBLIC ${TensorRT_INCLUDE_DIRS}
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
elseif(MODEL_RUNNER MATCHES "openvino.*$")
    find_package(ngraph REQUIRED)
    find_package(InferenceEngine REQUIRED)
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC MODEL_RUNNER_OPENVINO
    )
    target_link_libraries( ${PROJECT_NAME}
        PRIVATE
            ${InferenceEngine_LIBRARIES}
            ${NGRAPH_LIBRARIES}
    )
    target_sources(${PROJECT_NAME} PRIVATE openvino/vino_util.cpp)
    if(MODEL_RUNNER STREQUAL "openvino_sjtu")
        target_sources(${PROJECT_NAME} PRIVATE openvino/VinoModelSjtu.cpp)
    elseif(MODEL_RUNNER STREQUAL "openvino_nanodet")
        target_sources(${PROJECT_NAME} PRIVATE openvino/VinoModelNanodet.cpp)
    elseif(MODEL_RUNNER STREQUAL "openvino_yolox")
        target_sources(${PROJECT_NAME} PRIVATE openvino/VinoModelYolox.cpp)
    endif()
else()
    message(FATAL_ERROR "未知 MODEL_RUNNER : " ${MODEL_RUNNER})
endif()
