#ifndef __RM_COMMON_LOGGING_HPP__
#define __RM_COMMON_LOGGING_HPP__

#include <chrono>
#include <rclcpp/logging.hpp>
#include <string_view>

#define __LOG_ERROR(fmsg, ...) RCLCPP_ERROR(rclcpp::get_logger(__FILE__), fmsg, ##__VA_ARGS__)

#define __LOG_WARNING(fmsg, ...) RCLCPP_WARN(rclcpp::get_logger(__FILE__), fmsg, ##__VA_ARGS__)

#define __LOG_INFO(fmsg, ...) RCLCPP_INFO(rclcpp::get_logger(__FILE__), fmsg, ##__VA_ARGS__)

#define __LOG_DEBUG(fmsg, ...) RCLCPP_DEBUG(rclcpp::get_logger(__FILE__), fmsg, ##__VA_ARGS__)

#define __LOG_ERROR_AND_EXIT(fmsg, ...) \
  {                                     \
    __LOG_ERROR(fmsg, ##__VA_ARGS__);   \
    exit(1);                            \
  }

#endif