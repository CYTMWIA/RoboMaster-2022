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

  bool operator==(const BoundingBox& bbox) const
  {
    return confidence == bbox.confidence && color_id == bbox.color_id && tag_id == bbox.tag_id &&
           pts[0] == bbox.pts[0] && pts[1] == bbox.pts[1] && pts[2] == bbox.pts[2] &&
           pts[3] == bbox.pts[3];
  }

  bool operator!=(const BoundingBox& bbox) const { return !operator==(bbox); }
};
}  // namespace rm_detect

#endif