#include <rclcpp/rclcpp.hpp>

#include "rm_common/logging.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  __LOG_DEBUG("rclcpp init done");

  return 0;
}
