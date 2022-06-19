#ifndef __DETECT_OCV_POWER_RUNE_DETECTOR_HPP__
#define __DETECT_OCV_POWER_RUNE_DETECTOR_HPP__

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "collections.hpp"

namespace rm_detect
{
    class OcvPowerRuneDetector
    {
    private:
        class Impl;
        std::unique_ptr<Impl> pimpl_;
    public:
        OcvPowerRuneDetector(/* args */);
        ~OcvPowerRuneDetector();

        std::vector<Armor> operator()(const cv::Mat& src);
    };
} // namespace rm_detect



#endif