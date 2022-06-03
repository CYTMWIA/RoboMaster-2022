#ifndef __RM_AUTOAIM_CAPTURE_NODE_HPP__
#define __RM_AUTOAIM_CAPTURE_NODE_HPP__

#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>

#include "base_capture.hpp"

namespace rm_autoaim
{
class CaptureNode : public rclcpp::Node
{
 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::thread thread_;
  std::unique_ptr<cv::VideoCapture> pvideocapture_;
  std::unique_ptr<BaseCameraCapture> pcamera_;

 public:
  /**
   * @brief 构造 CaptureNode 捕获 图像、视频 或其他 VideoCapture 支持的图像来源
   *
   * @tparam T
   * @param arg_to_videocapture
   * @param output
   * @param node_name
   */
  template <typename T>
  explicit CaptureNode(const T &arg_to_videocapture, const std::string &output,
                       const std::string &node_name = "capture_node")
      : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    pvideocapture_ = std::make_unique<cv::VideoCapture>(arg_to_videocapture);
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, rclcpp::SensorDataQoS());
    thread_ = std::thread(std::bind(&CaptureNode::loop_videocapture, this));
  }

  /**
   * @brief 构造 CaptureNode 捕获 工业相机（大恒、大华）
   *
   * @param settings
   * @param node_name
   */
  explicit CaptureNode(const CameraSettings &settings, const std::string &output,
                       const std::string &node_name = "capture_node");

 private:
  void loop_videocapture();

  void init_camera(const CameraSettings &settings);
  void loop_camera(const CameraSettings settings);
};
}  // namespace rm_autoaim

#endif