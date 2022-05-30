#ifndef __DETECT_CVARMORDETECTOR_HPP__
#define __DETECT_CVARMORDETECTOR_HPP__

#include "MatchsClassifier.hpp"
#include "BoundingBox.hpp"
#include "cv_util.hpp"

#include <opencv2/opencv.hpp>

#include <vector>

namespace rmcv::detect
{
    class CvArmorDetector
    {
    private:
        MatchsClassifier armor_classifier_;

        int32_t match_armor_icon(const cv::Mat &img, const Lightbar &left, const Lightbar &right, const double &thresh);
    public:
        CvArmorDetector();

        std::vector<BoundingBox> operator()(const cv::Mat &src);
    };
}


#endif