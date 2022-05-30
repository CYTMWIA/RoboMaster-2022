get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_DIR})
if (TARGET ${PROJECT_NAME})
    return()
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})


find_package(OpenCV 4 REQUIRED)
find_package(fmt REQUIRED)

add_subdirectory(../logging logging_binary_dir)
add_subdirectory(../threading threading_dir) # Debug

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}
)

target_include_directories( ${PROJECT_NAME}
    SYSTEM PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${fmt_INCLUDE_DIRS}
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        Model.cpp
        cv_armor.cpp
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        logging
        threading
        fmt::fmt
)

if(NOT DEFINED MODEL_RUNNER)
    set(MODEL_RUNNER "openvino_yolox")
endif()
message("当前 MODEL_RUNNER : " ${MODEL_RUNNER})

if(MODEL_RUNNER MATCHES "tensorrt.*$")
    find_package(TensorRT REQUIRED)
    target_compile_definitions( ${PROJECT_NAME}
        PUBLIC USE_TENSORRT_SJTU
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
        PUBLIC USE_OPENVINO
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
