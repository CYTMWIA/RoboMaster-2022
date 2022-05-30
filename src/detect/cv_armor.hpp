#ifndef __CV_ARMOR_HPP__
#define __CV_ARMOR_HPP__

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
    };

    RRect min_area_rect(cv::InputArray points);

   void fix_boundingbox(BoundingBox &bbox, const cv::Mat &img);
}

#endif