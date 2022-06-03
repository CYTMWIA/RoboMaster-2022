#ifndef __DETECT_CV_ARMOR_UTIL_HPP__
#define __DETECT_CV_ARMOR_UTIL_HPP__

#include "bounding_box.hpp"
#include "cv_util.hpp"

namespace rm_autoaim
{
struct LightbarMatchResult
{
  int left_idx, right_idx;  // 左右灯条序号
  double confidence;        // 置信度
};

double judge_lightbars_pair(const BoundingBox &lightbars_bbox);

double judge_lightbars_pair(const RRect &left, const RRect &right);

void match_lightbars(std::vector<RRect> &rrects, std::vector<LightbarMatchResult> &results);

void fix_boundingbox(BoundingBox &bbox, const cv::Mat &img);
}  // namespace rm_autoaim

#endif