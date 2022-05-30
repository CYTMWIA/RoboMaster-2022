#include "NnArmorDetector.hpp"

#include "cv_util.hpp"

#include "logging/logging.hpp"
#include "threading/threading.hpp"

#include <algorithm>

namespace rmcv::detect
{
    NnArmorDetector::NnArmorDetector(const std::string &model_path): model_(model_path)
    {
    }

    std::vector<BoundingBox> NnArmorDetector::operator()(const cv::Mat &frame)
    {
        auto detect_result = model_(frame);
        for (auto& det: detect_result)
        {
            fix_boundingbox(det, frame);
        }

        std::vector<bool> founds(last_detect_result_.size(), 0);
        for (int i=0;i<founds.size();i++)
        {
            auto& det_prev = last_detect_result_[i];
            for (auto& det: detect_result)
            {
                if (
                    // det_prev.color_id==det.color_id
                    // && det_prev.tag_id==det.tag_id
                     (to_rect(det_prev) & to_rect(det)).area()>0)
                {
                    // __LOG_DEBUG("FOUND");
                    founds[i] = true;
                    break;
                }
            }
        }

        for (int i=0;i<founds.size();i++)
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
                for (const auto& s: status)
                {
                    if (s!=1)
                    {
                        all_good_flag = false;
                        break;
                    }
                }

                if (all_good_flag)
                {
                    bool bad_flag = false;
                    double max_dis = std::max(1.0, 0.15*distance(prev_pts[0],prev_pts[2]));
                    for (int i=0;i<4;i++)
                        if (rmcv::detect::distance(prev_pts[i], next_pts[i])>max_dis)
                        {
                            bad_flag = true;
                            break;
                        }
                    if (bad_flag) continue;

                    for (int i=0;i<4;i++)
                        det_prev.pts[i] = next_pts[i];
                    detect_result.push_back(det_prev);
                }
            }
        }
        // __LOG_DEBUG("COUNT {}", detect_result.size());
        // for (int i=0;i<detect_result.size();i++)
        // {
            // if (i==0)
            //     rmcv::threading::RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {
            //         detect_result[i].pts[0].x, detect_result[i].pts[0].y,
            //         detect_result[i].pts[1].x, detect_result[i].pts[1].y,
            //         detect_result[i].pts[2].x, detect_result[i].pts[2].y,
            //         detect_result[i].pts[3].x, detect_result[i].pts[3].y
            //     });
            // __LOG_DEBUG(
            //     "({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f})",
            //     detect_result[i].pts[0].x, detect_result[i].pts[0].y,
            //     detect_result[i].pts[1].x, detect_result[i].pts[1].y,
            //     detect_result[i].pts[2].x, detect_result[i].pts[2].y,
            //     detect_result[i].pts[3].x, detect_result[i].pts[3].y
            // );
        // }
        last_detect_result_ = detect_result;
        last_frame_ = frame;
        return detect_result;   
    }
}