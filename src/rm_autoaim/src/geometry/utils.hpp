#ifndef __GEOMETRY_UTILS_HPP__
#define __GEOMETRY_UTILS_HPP__

#include <cmath>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace rm_autoaim
{

inline double to_angle(double rad) { return rad * (180.0 / M_PI); }
inline double to_rad(double angle) { return angle * (M_PI / 180.0); }

inline double distance(const cv::Point2f& from, const cv::Point2f& to)
{
  return cv::norm(to - from);
}

inline double included_angle(const cv::Point2f& center, const cv::Point2f& from,
                             const cv::Point2f& to)
{
  auto v1 = from - center;
  auto v2 = to - center;
  return to_angle(acos(v1.dot(v2) / abs(cv::norm(v1) * cv::norm(v2))));
}

}  // namespace rm_autoaim

#endif