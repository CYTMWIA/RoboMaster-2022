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

    struct LightbarMatchResult
    {
        int left_idx, right_idx; // 左右灯条序号
        double confidence;       // 置信度
    };

    cv::Mat awakenlion_threshold(const cv::Mat &src, double gray_max_ = 0.6, double gray_avg_ = 0.4, double ch0 = 0.1, double ch1 = 0.2, double ch2 = 0.7);

    std::vector<std::vector<cv::Point>> find_external_contours(const cv::Mat &src);

    cv::Mat safe_range_roi(const cv::Mat &src, cv::Range range_rows, cv::Range range_cols);

    BoundingBox make_boundingbox(const RRect &left, const RRect &right);

    BoundingBox make_boundingbox(const RRect &rrect);

    template<typename T>
    void calc_gray_hist(const cv::Mat &src, T* res)
    {
        for (int i=0;i<256;i++) res[i] = 0;
        for (int i=0;i<src.rows;i++)
            for (int j=0;j<src.cols;j++)
                res[*(src.data+i*src.cols+j)] += 1;
    }

    inline double distance(cv::Point2f p1, cv::Point2f p2)
    {
        return abs(cv::norm(p1 - p2));
    }

    inline cv::Rect2f to_bounding_rect(const BoundingBox& bbox)
    {
        return cv::boundingRect(std::vector<cv::Point2f>{&bbox.pts[0], &bbox.pts[4]});
    }
}

#endif