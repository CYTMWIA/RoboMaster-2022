#ifndef __COMMON_INTERFACES_HPP__
#define __COMMON_INTERFACES_HPP__

#include <chrono>
#include <opencv2/opencv.hpp>
#include <type_traits>

namespace rmc::interfaces
{

inline auto now() { return std::chrono::steady_clock::now(); }

using Timestamp = std::invoke_result<decltype(now)>::type;

struct Rpy
{
  double roll, pitch, yaw;
};

struct CaptureImage
{
  Timestamp stamp;
  cv::Mat image;
  Rpy rpy;
};

}  // namespace rm_interfaces

#endif