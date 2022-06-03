cmake_minimum_required(VERSION 3.5)
project(rm_detect)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2
find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Others
find_package(OpenCV REQUIRED)
# Ours
find_package(rm_common REQUIRED)
find_package(rm_interfaces REQUIRED)

add_library(rm_detect
  src/detect_node.cpp
  src/cv_util.cpp
  src/cv_armor_util.cpp
  src/hamming_match.cpp
  src/matchs_classifier.cpp
  src/cv_armor_detector.cpp
  src/nn_armor_detector.cpp
  src/lk_optical_flow_tracker.cpp
)

ament_target_dependencies(rm_detect
  builtin_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  rm_common
  rm_interfaces
)

target_include_directories(rm_detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 神经网络：
#   前缀为使用的加速库：none, openvino, tensorrt
#   后缀为实际使用的模型
if(NOT DEFINED ARMOR_MODEL)
    set(ARMOR_MODEL "none")
endif()
message("当前 ARMOR_MODEL : " ${ARMOR_MODEL})

if(ARMOR_MODEL MATCHES "none")
  target_compile_definitions( rm_detect
    PUBLIC ARMOR_MODEL_NONE
  )
elseif(ARMOR_MODEL MATCHES "tensorrt.*$")
  find_package(TensorRT REQUIRED)
  target_compile_definitions( rm_detect
    PUBLIC ARMOR_MODEL_TENSORRT_SJTU
  )
  target_include_directories( rm_detect
    SYSTEM PUBLIC ${TensorRT_INCLUDE_DIRS}
  )
  target_link_libraries( rm_detect
    PRIVATE
      ${TensorRT_LIBS}
      nvonnxparser
      cuda
  )
  target_sources( rm_detect
    PRIVATE
      tensorrt_sjtu/trtlogger.cpp
      tensorrt_sjtu/trtmodule.cpp
  )
elseif(ARMOR_MODEL MATCHES "openvino.*$")
  find_package(ngraph REQUIRED)
  find_package(InferenceEngine REQUIRED)
  target_compile_definitions( rm_detect
    PUBLIC ARMOR_MODEL_OPENVINO
  )
  target_link_libraries( rm_detect
    PRIVATE
      ${InferenceEngine_LIBRARIES}
      ${NGRAPH_LIBRARIES}
  )
  target_sources(rm_detect PRIVATE src/openvino/vino_util.cpp)
  if(ARMOR_MODEL STREQUAL "openvino_sjtu")
    target_sources(rm_detect PRIVATE src/openvino/vino_model_sjtu.cpp)
  elseif(ARMOR_MODEL STREQUAL "openvino_nanodet")
    target_sources(rm_detect PRIVATE src/openvino/vino_model_nanodet.cpp)
  elseif(ARMOR_MODEL STREQUAL "openvino_yolox")
    target_sources(rm_detect PRIVATE src/openvino/vino_model_yolox.cpp)
  endif()
else()
  message(FATAL_ERROR "未知 ARMOR_MODEL : " ${ARMOR_MODEL})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rm_detect
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  builtin_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  rm_common
  rm_interfaces
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  rm_detect
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
