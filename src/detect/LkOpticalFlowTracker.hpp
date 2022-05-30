#ifndef __DETECT_LKOPTICALFLOWTRACKER_HPP__
#define __DETECT_LKOPTICALFLOWTRACKER_HPP__

#include "BoundingBox.hpp"

#include <opencv2/opencv.hpp>

#include <vector>

namespace rmcv::detect
{
    class LkOpticalFlowTracker
    {
    private:
        cv::Mat last_frame_;
        std::vector<BoundingBox> last_detect_result_;
    public:
        void track(const cv::Mat& frame, std::vector<BoundingBox> &detect_result);
    };
}

#endif