#ifndef __DETECT_MATCHSCLASSIFIER_HPP__
#define __DETECT_MATCHSCLASSIFIER_HPP__

#include "HammingMatch.hpp"

#include <opencv2/opencv.hpp>

#include <unordered_map>

namespace rmcv::detect
{
    class MatchsClassifier
    {
    private:
        std::unordered_map<int, HammingMatch> matchs;

    public:
        void add_match(int32_t id, HammingMatch match);
        int32_t operator()(const cv::Mat &src, double thresh = 0.66);
    };
}

#endif