#ifndef __RM_COMMON_IMAGE_MSG_HPP__
#define __RM_COMMON_IMAGE_MSG_HPP__

#include <chrono>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace rm_common
{
void set_now(builtin_interfaces::msg::Time &time)
{
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  if (now <= std::chrono::nanoseconds(0))
  {
    time.sec = time.nanosec = 0;
  }
  else
  {
    time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    time.nanosec = now.count() % 1000000000;
  }
}

int encoding2mat_type(const std::string &encoding)
{
  if (encoding == "mono8")
  {
    return CV_8UC1;
  }
  else if (encoding == "bgr8")
  {
    return CV_8UC3;
  }
  else if (encoding == "mono16")
  {
    return CV_16SC1;
  }
  else if (encoding == "rgba8")
  {
    return CV_8UC4;
  }
  throw std::runtime_error("Unsupported mat type");
}

std::string mat_type2encoding(int mat_type)
{
  switch (mat_type)
  {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void make_image_msg(cv::Mat &src, sensor_msgs::msg::Image::UniquePtr &msg,
                    std::string frame_id = "frame")
{
  set_now(msg->header.stamp);
  msg->header.frame_id = frame_id;
  msg->height = src.rows;
  msg->width = src.cols;
  msg->encoding = mat_type2encoding(src.type());
  msg->is_bigendian = false;
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(src.step);
  msg->data.assign(src.datastart, src.dataend);
}
}  // namespace rm_common

#endif  // RM_COMMON___RM_COMMON_HPP_
