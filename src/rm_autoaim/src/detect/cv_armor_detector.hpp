#ifndef __DETECT_CV_ARMOR_DETECTOR_HPP__
#define __DETECT_CV_ARMOR_DETECTOR_HPP__

#include <opencv2/opencv.hpp>
#include <vector>

#include "bounding_box.hpp"
#include "cv_util.hpp"
#include "lk_optical_flow_tracker.hpp"
#include "matchs_classifier.hpp"

namespace rm_autoaim
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
}  // namespace rm_autoaim

#endif