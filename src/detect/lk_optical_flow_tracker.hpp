#ifndef __DETECT_LK_OPTICAL_FLOW_TRACKER_HPP__
#define __DETECT_LK_OPTICAL_FLOW_TRACKER_HPP__

#include <opencv2/opencv.hpp>
#include <vector>

#include "bounding_box.hpp"

namespace rmcv::detect
{
class LkOpticalFlowTracker
{
 private:
  cv::Mat last_frame_;
  std::vector<BoundingBox> last_detect_result_;

 public:
  void track(const cv::Mat& frame, std::vector<BoundingBox>& detect_result);
};
}  // namespace rmcv::detect

#endif