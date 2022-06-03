#include "rm_detect/nn_armor_detector.hpp"

#include <algorithm>

#include "rm_common/logging.hpp"
#include "rm_detect/cv_armor_util.hpp"
#include "rm_detect/cv_util.hpp"

namespace rm_detect
{
NnArmorDetector::NnArmorDetector(const std::string &model_path) : model_(model_path) {}

std::vector<BoundingBox> NnArmorDetector::operator()(const cv::Mat &frame)
{
  auto detect_result = model_(frame);
  for (auto &det : detect_result)
  {
    fix_boundingbox(det, frame);
  }

  tracker_.track(frame, detect_result);

  return detect_result;
}
}  // namespace rm_detect