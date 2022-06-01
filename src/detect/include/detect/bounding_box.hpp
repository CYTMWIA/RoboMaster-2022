#ifndef __DETECT_BOUNDING_BOX_HPP__
#define __DETECT_BOUNDING_BOX_HPP__

#include <opencv2/opencv.hpp>

namespace rm_detect
{
struct alignas(4) BoundingBox
{
  cv::Point2f pts[4];  // [pt0, pt1, pt2, pt3] 左上 左下 右下 右上
  float confidence;
  int color_id;  // 0: blue, 1: red, 2: gray
  int tag_id;    // 0: guard, 1-5: number, 6: base

  bool operator==(const BoundingBox&) const = default;
  bool operator!=(const BoundingBox&) const = default;
};
}  // namespace rm_detect

#endif