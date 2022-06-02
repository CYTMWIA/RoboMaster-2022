#ifndef __RM_DETECT_CV_ARMOR_DETECTOR_HPP__
#define __RM_DETECT_CV_ARMOR_DETECTOR_HPP__

#include <opencv2/opencv.hpp>
#include <vector>

#include "cv_util.hpp"
#include "lk_optical_flow_tracker.hpp"
#include "matchs_classifier.hpp"
#include "rm_detect/bounding_box.hpp"

namespace rm_detect
{
class CvArmorDetector
{
 private:
  MatchsClassifier armor_classifier_;

  LkOpticalFlowTracker tracker_;

  int32_t match_armor_icon(const cv::Mat &img, const BoundingBox &armor_bbox, const double &thresh);

 public:
  CvArmorDetector();

  std::vector<BoundingBox> operator()(const cv::Mat &src);
};
}  // namespace rm_detect

#endif