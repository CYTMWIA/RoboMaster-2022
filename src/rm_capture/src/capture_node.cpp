#include "rm_capture/capture_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include "rm_capture/daheng_capture.hpp"
#include "rm_capture/dahua_capture.hpp"
#include "rm_common/image_msg.hpp"
#include "rm_common/logging.hpp"

namespace rm_capture
{
CaptureNode::CaptureNode(const CameraSettings &settings, const std::string &output,
                         const std::string &node_name)
    : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, rclcpp::SensorDataQoS());
  thread_ = std::thread(std::bind(&CaptureNode::loop_camera, this, settings));
}

void CaptureNode::loop_videocapture()
{
  cv::Mat frame;
  while (rclcpp::ok())
  {
    *pvideocapture_ >> frame;
    if (frame.empty())
    {
      __LOG_WARNING("获取帧失败");
      continue;
    }

    sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
    rm_common::make_image_msg(frame, msg);
    pub_->publish(std::move(msg));
  }
}

void CaptureNode::init_camera(const CameraSettings &settings)
{
  if (settings.manufacturer == "daheng")
    pcamera_ = std::make_unique<DahengCapture>();
  else if (settings.manufacturer == "dahua")
    pcamera_ = std::make_unique<DahuaCapture>();
  else
  {
    __LOG_ERROR("未知的相机厂商 {}", settings.manufacturer);
    return;
  }

  pcamera_->set_exposure_time(settings.exposure_time);
  pcamera_->set_gain(settings.gain);
  pcamera_->set_white_balance(settings.white_balance_red, settings.white_balance_green,
                              settings.white_balance_blue);
}

void CaptureNode::loop_camera(const CameraSettings settings)
{
  init_camera(settings);
  cv::Mat frame;
  while (rclcpp::ok())
  {
    try
    {
      frame = pcamera_->next();
    }
    catch (const std::exception &e)
    {
      __LOG_ERROR("捕获图像失败 {}", e.what());
      continue;
    }

    sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
    rm_common::make_image_msg(frame, msg);
    pub_->publish(std::move(msg));
  }
}
}  // namespace rm_capture

int main(int argc, char **argv)
{
  using namespace rm_capture;

  rclcpp::init(argc, argv);
  std::shared_ptr<CaptureNode> cap_node;
  CameraSettings cs;
  cs.manufacturer = "dahua";
  cs.exposure_time = 9940;
  cs.gain = 1.6;
  cs.white_balance_blue = cs.white_balance_green = cs.white_balance_red = 1;
  try
  {
    cap_node = std::make_shared<CaptureNode>(cs, "capture_frame");
  }
  catch (const std::exception &e)
  {
    __LOG_ERROR("创建节点失败 {}", e.what());
    return 1;
  }

  rclcpp::spin(cap_node);

  rclcpp::shutdown();

  return 0;
}
