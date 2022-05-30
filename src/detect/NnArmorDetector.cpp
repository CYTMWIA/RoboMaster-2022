#include "NnArmorDetector.hpp"

#include "cv_util.hpp"
#include "cv_armor_util.hpp"

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

        tracker_.track(frame, detect_result);

        return detect_result;   
    }
}