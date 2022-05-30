#ifndef __DETECT_CV_UTIL_HPP__
#define __DETECT_CV_UTIL_HPP__

#include <opencv2/opencv.hpp>

#include "BoundingBox.hpp"

namespace rmcv::detect
{
    struct RRect
    {
        /*
         * width始终为长边，angle始终为长边从 向正右方射线 沿逆时针转动的角度即[0, 180)
         */
        cv::Point2f center;
        float width, height, rad;

        RRect() = default;
        RRect(const cv::RotatedRect &cvrrect);
    };

    struct Lightbar
    {
        cv::Point2f vertex_up;   // 上顶点
        cv::Point2f vertex_down; // 下顶点

        Lightbar() = default;
        Lightbar(const RRect &rrect, const double &scale=1);
        Lightbar extend(double scale) const;
    };

    struct LightbarMatchResult
    {
        int left_idx, right_idx; // 左右灯条序号
        double confidence;       // 置信度
    };

    inline double distance(cv::Point2f p1, cv::Point2f p2);

    cv::Mat awakenlion_threshold(const cv::Mat &src, double gray_max_ = 0.6, double gray_avg_ = 0.4, double ch0 = 0.1, double ch1 = 0.2, double ch2 = 0.7);

    void fix_boundingbox(BoundingBox &bbox, const cv::Mat &img);

    void match_lightbars(std::vector<RRect> &rrects, std::vector<Lightbar> &lightbars, std::vector<LightbarMatchResult> &results);
}

#endif