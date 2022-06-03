#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "rm_capture/capture_node.hpp"
#include "rm_common/logging.hpp"
#include "rm_detect/detect_node.hpp"

int main(int argc, char **argv)
{
  using namespace rm_capture;
  using namespace rm_detect;
  rclcpp::init(argc, argv);

  __LOG_INFO("Working Dir:");
  system("pwd");

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<CaptureNode> cap_node;
  CameraSettings cs;
  cs.manufacturer = "dahua";
  cs.exposure_time = 9940;
  cs.gain = 1.6;
  cs.white_balance_blue = cs.white_balance_green = cs.white_balance_red = 1;
  try
  {
    cap_node = std::make_shared<CaptureNode>(cs, "capture_frame");
    executor.add_node(cap_node);
  }
  catch (const std::exception &e)
  {
    __LOG_ERROR("创建 CaptureNode 失败 {}", e.what());
    return 1;
  }

  std::shared_ptr<rm_detect::DetectNode> det_node;
  try
  {
    det_node = std::make_shared<rm_detect::DetectNode>("capture_frame", "detect_result");
    executor.add_node(det_node);
  }
  catch(const std::exception& e)
  {
    __LOG_ERROR("创建 DetectNode 失败 {}", e.what());
  }

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
