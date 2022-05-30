#ifndef __DETECT_CV_ARMOR_UTIL_HPP__
#define __DETECT_CV_ARMOR_UTIL_HPP__

#include "BoundingBox.hpp"
#include "cv_util.hpp"

namespace rmcv::detect
{
    double judge_lightbars_pair(const BoundingBox &lightbars_bbox);

    double judge_lightbars_pair(const RRect &left, const RRect &right);

    void match_lightbars(std::vector<RRect> &rrects, std::vector<LightbarMatchResult> &results);

    void fix_boundingbox(BoundingBox &bbox, const cv::Mat &img);
}


#endif