#ifndef __DETECT_COLLECTIONS_HPP__
#define __DETECT_COLLECTIONS_HPP__

#include <opencv2/opencv.hpp>

#include "common/data.hpp"

namespace rm_detect
{
/**
 * @brief 装甲板检测结果
 *
 */
struct Armor
{
  cv::Point2f pts[4];  // [pt0, pt1, pt2, pt3] 左上 左下 右下 右上
  int color_id;        // 0: blue, 1: red, 2: gray
  int tag_id;          // 0: guard, 1-5: number, 6: base
  rm_data::ArmorType type;
};

}  // namespace rm_detect

#endif