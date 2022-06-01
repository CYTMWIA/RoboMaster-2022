#include "detect/lk_optical_flow_tracker.hpp"

#include "detect/cv_armor_util.hpp"
#include "detect/cv_util.hpp"

namespace rm_detect
{
void LkOpticalFlowTracker::track(const cv::Mat& frame, std::vector<BoundingBox>& detect_result)
{
  std::vector<bool> founds(last_detect_result_.size(), 0);
  for (int i = 0; i < founds.size(); i++)
  {
    auto& det_prev = last_detect_result_[i];
    for (auto& det : detect_result)
    {
      if (
          // det_prev.color_id==det.color_id
          // && det_prev.tag_id==det.tag_id
          (to_bounding_rect(det_prev) & to_bounding_rect(det)).area() > 0)
      {
        // __LOG_DEBUG("FOUND");
        founds[i] = true;
        break;
      }
    }
  }

  for (int i = 0; i < founds.size(); i++)
  {
    if (!founds[i])
    {
      auto& det_prev = last_detect_result_[i];
      std::vector<cv::Point2f> prev_pts{&det_prev.pts[0], &det_prev.pts[4]};
      std::vector<cv::Point2f> next_pts;
      std::vector<uint8_t> status;
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(last_frame_, frame, prev_pts, next_pts, status, err);

      bool all_good_flag = true;
      for (const auto& s : status)
      {
        if (s != 1)
        {
          all_good_flag = false;
          break;
        }
      }

      if (all_good_flag)
      {
        bool bad_flag = false;
        double max_dis = std::max(1.0, 0.5 * distance(prev_pts[0], prev_pts[2]));  // 最大变化距离
        for (int i = 0; i < 4; i++)
          if (rm_detect::distance(prev_pts[i], next_pts[i]) > max_dis)
          {
            bad_flag = true;
            break;
          }
        if (bad_flag) continue;

        BoundingBox bbox;
        for (int i = 0; i < 4; i++) bbox.pts[i] = next_pts[i];
        auto c = judge_lightbars_pair(bbox);
        // std::cout << "TC: " << c << std::endl;
        if (c < 0.6) continue;

        for (int i = 0; i < 4; i++) det_prev.pts[i] = next_pts[i];
        detect_result.push_back(det_prev);
      }
    }
  }

  last_detect_result_ = detect_result;
  last_frame_ = frame;
}
}  // namespace rm_detect