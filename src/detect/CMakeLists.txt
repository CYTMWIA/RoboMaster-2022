get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

add_library(${PROJECT_NAME})

target_sources( ${PROJECT_NAME}
    PRIVATE
        cv_util.cpp
        cv_armor_util.cpp
        hamming_match.cpp
        matchs_classifier.cpp
        cv_armor_detector.cpp
        nn_armor_detector.cpp
        bounding_box.cpp
        lk_optical_flow_tracker.cpp
)

# 神经网络：
#   前缀为使用的加速库：none, openvino, tensorrt
#   后缀为实际使用的模型
if(NOT DEFINED ARMOR_MODEL)
    set(ARMOR_MODEL "none")
endif()
message("当前 ARMOR_MODEL : " ${ARMOR_MODEL})

if(ARMOR_MODEL MATCHES "none")
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC ARMOR_MODEL_NONE
    )
elseif(ARMOR_MODEL MATCHES "tensorrt.*$")
    find_package(TensorRT REQUIRED)
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC ARMOR_MODEL_TENSORRT_SJTU
    )
    target_include_directories( ${PROJECT_NAME}
        SYSTEM PUBLIC ${TensorRT_INCLUDE_DIRS}
    )
    target_link_libraries( ${PROJECT_NAME}
        PRIVATE
            ${TensorRT_LIBS}
            nvonnxparser
            cuda
    )
    target_sources( ${PROJECT_NAME}
        PRIVATE
            tensorrt_sjtu/trtlogger.cpp
            tensorrt_sjtu/trtmodule.cpp
    )
elseif(ARMOR_MODEL MATCHES "openvino.*$")
    find_package(ngraph REQUIRED)
    find_package(InferenceEngine REQUIRED)
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC ARMOR_MODEL_OPENVINO
    )
    target_link_libraries( ${PROJECT_NAME}
        PRIVATE
            ${InferenceEngine_LIBRARIES}
            ${NGRAPH_LIBRARIES}
    )
    target_sources(${PROJECT_NAME} PRIVATE openvino/vino_util.cpp)
    if(ARMOR_MODEL STREQUAL "openvino_sjtu")
        target_sources(${PROJECT_NAME} PRIVATE openvino/vino_model_sjtu.cpp)
    elseif(ARMOR_MODEL STREQUAL "openvino_nanodet")
        target_sources(${PROJECT_NAME} PRIVATE openvino/vino_model_nanodet.cpp)
    elseif(ARMOR_MODEL STREQUAL "openvino_yolox")
        target_sources(${PROJECT_NAME} PRIVATE openvino/vino_model_yolox.cpp)
    endif()
else()
    message(FATAL_ERROR "未知 ARMOR_MODEL : " ${ARMOR_MODEL})
endif()
