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

  std::shared_ptr<CaptureNode> wide_node;

  CameraSettings cs;
  cs.manufacturer = "dahua";
  cs.exposure_time = 9940;
  cs.gain = 1.6;
  cs.white_balance_blue = 1.7;
  cs.white_balance_green = 1.0;
  cs.white_balance_red = 1.6;
  try
  {
    wide_node = std::make_shared<CaptureNode>(cs, "wide_frame");
    executor.add_node(wide_node);
  }
  catch (const std::exception &e)
  {
    __LOG_ERROR("创建 全场视角节点 失败 {}", e.what());
    return 1;
  }

  std::shared_ptr<CaptureNode> far_node;
  try
  {
    far_node = std::make_shared<CaptureNode>(0, "far_frame");
  }
  catch (const std::exception &e)
  {
    __LOG_ERROR("创建 敌方基地视角节点 失败 {}", e.what());
    return 1;
  }

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
