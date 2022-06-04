#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "config.hpp"
#include "rm_autoaim/capture_node.hpp"
#include "rm_common/logging.hpp"

int main(int argc, char **argv)
{
  using namespace rm_autoaim;
  using namespace rm_bringup;

  rclcpp::init(argc, argv);
  __LOG_INFO("RCLCPP Init Done.");

  config::init();

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<CaptureNode> cap_node;

  CameraSettings cs;
  cs.manufacturer = config::get<std::string>("daheng", "camera", "manufacturer");
  cs.exposure_time = config::get<config::Float>(9940, "camera", "exposure_time");
  cs.gain = config::get<config::Float>(1.6, "camera", "gain");
  cs.white_balance_blue = config::get<config::Float>(1.7, "camera", "white_balance_blue");
  cs.white_balance_green = config::get<config::Float>(1.0, "camera", "white_balance_green");
  cs.white_balance_red = config::get<config::Float>(1.6, "camera", "white_balance_red");
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

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
