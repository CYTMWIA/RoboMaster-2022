#ifndef __OCV_UTILS_HPP__
#define __OCV_UTILS_HPP__
#include <opencv2/opencv.hpp>
#include <vector>

namespace rm_detect
{
inline std::vector<std::vector<cv::Point>> find_external_contours(const cv::Mat &src)
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;  // unused
  cv::findContours(src, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  return contours;
}

}  // namespace rm_detect

#endif