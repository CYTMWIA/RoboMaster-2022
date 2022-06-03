cmake_minimum_required(VERSION 3.5)
project(rm_autoaim)

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


# 构建工具
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# ROS2
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# 第三方库
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# 项目内其它 package
find_package(rm_common REQUIRED)
find_package(rm_interfaces REQUIRED)

file(GLOB sources
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/capture/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/detect/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/predict/*.cpp
)
add_library(rm_autoaim ${sources})

ament_target_dependencies(rm_autoaim
  rclcpp
  builtin_interfaces
  sensor_msgs
  std_msgs
  OpenCV
  Eigen3
  rm_common
  rm_interfaces
)

target_include_directories(rm_autoaim
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/daheng/include
  /opt/HuarayTech/MVviewer/include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(rm_autoaim
  ${CMAKE_SOURCE_DIR}/3rdparty/daheng/lib/libgxiapi.so
  /opt/HuarayTech/MVviewer/lib/libMVSDK.so
)

# 神经网络：
#   前缀为使用的加速库：none, openvino, tensorrt
#   后缀为实际使用的模型
if(NOT DEFINED ARMOR_MODEL)
    set(ARMOR_MODEL "none")
endif()
message("当前 ARMOR_MODEL : " ${ARMOR_MODEL})
if(ARMOR_MODEL MATCHES "none")
  target_compile_definitions( rm_autoaim
    PUBLIC ARMOR_MODEL_NONE
  )
elseif(ARMOR_MODEL MATCHES "tensorrt.*$")
  find_package(TensorRT REQUIRED)
  target_compile_definitions( rm_autoaim
    PUBLIC ARMOR_MODEL_TENSORRT_SJTU
  )
  target_include_directories( rm_autoaim
    SYSTEM PUBLIC ${TensorRT_INCLUDE_DIRS}
  )
  target_link_libraries( rm_autoaim
    PRIVATE
      ${TensorRT_LIBS}
      nvonnxparser
      cuda
  )
  target_sources( rm_autoaim
    PRIVATE
      tensorrt_sjtu/trtlogger.cpp
      tensorrt_sjtu/trtmodule.cpp
  )
elseif(ARMOR_MODEL MATCHES "openvino.*$")
  find_package(ngraph REQUIRED)
  find_package(InferenceEngine REQUIRED)
  target_compile_definitions( rm_autoaim
    PUBLIC ARMOR_MODEL_OPENVINO
  )
  target_link_libraries( rm_autoaim
    PRIVATE
      ${InferenceEngine_LIBRARIES}
      ${NGRAPH_LIBRARIES}
  )
  target_sources(rm_autoaim PRIVATE src/openvino/vino_util.cpp)
  if(ARMOR_MODEL STREQUAL "openvino_sjtu")
    target_sources(rm_autoaim PRIVATE src/openvino/vino_model_sjtu.cpp)
  elseif(ARMOR_MODEL STREQUAL "openvino_nanodet")
    target_sources(rm_autoaim PRIVATE src/openvino/vino_model_nanodet.cpp)
  elseif(ARMOR_MODEL STREQUAL "openvino_yolox")
    target_sources(rm_autoaim PRIVATE src/openvino/vino_model_yolox.cpp)
  endif()
else()
  message(FATAL_ERROR "未知 ARMOR_MODEL : " ${ARMOR_MODEL})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rm_autoaim
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  sensor_msgs
  OpenCV
  rm_interfaces
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  rm_autoaim
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
