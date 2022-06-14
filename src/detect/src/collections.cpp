#include "detect/collections.hpp"

namespace rm_detect
{
RRect::RRect(const cv::RotatedRect& cvrrect)
{
  // 确定中心
  center = cvrrect.center;

  cv::Point2f pts[4];
  cvrrect.points(pts);

  // 确定长宽
  double len01 = sqrt((pts[0].x - pts[1].x) * (pts[0].x - pts[1].x) +
                      (pts[0].y - pts[1].y) * (pts[0].y - pts[1].y));
  double len03 = sqrt((pts[0].x - pts[3].x) * (pts[0].x - pts[3].x) +
                      (pts[0].y - pts[3].y) * (pts[0].y - pts[3].y));
  if (len01 < len03)
  {
    std::swap(pts[1], pts[3]);
    std::swap(len01, len03);
  }
  width = len01;
  height = len03;

  // 确定倾斜弧度
  if (pts[0].y > pts[1].y) std::swap(pts[0], pts[1]);
  rad = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
}
}  // namespace rm_detect
