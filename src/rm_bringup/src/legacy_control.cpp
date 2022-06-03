#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "rm_autoaim/capture_node.hpp"
#include "rm_autoaim/detect_node.hpp"
#include "rm_autoaim/predict_node.hpp"
#include "rm_common/logging.hpp"

int main(int argc, char **argv)
{
  using namespace rm_autoaim;
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

  std::shared_ptr<DetectNode> det_node;
  try
  {
    det_node = std::make_shared<DetectNode>("capture_frame", "detect_result");
    executor.add_node(det_node);
  }
  catch (const std::exception &e)
  {
    __LOG_ERROR("创建 DetectNode 失败 {}", e.what());
  }

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
