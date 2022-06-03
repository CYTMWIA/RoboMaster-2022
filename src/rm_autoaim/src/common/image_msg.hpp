#ifndef __COMMON_IMAGE_MSG_HPP__
#define __COMMON_IMAGE_MSG_HPP__

#include <chrono>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace rm_autoaim
{
void set_now(builtin_interfaces::msg::Time &time);

int64_t get_now();

int encoding2mat_type(const std::string &encoding);

std::string mat_type2encoding(int mat_type);

sensor_msgs::msg::Image image2msg(const cv::Mat &src, const std::string &frame_id = "frame");

cv::Mat msg2image(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
}  // namespace rm_autoaim

#endif  // RM_COMMON___RM_COMMON_HPP_
