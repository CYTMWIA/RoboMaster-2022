#ifndef __CV_ARMOR_HPP__
#define __CV_ARMOR_HPP__

#include <opencv2/opencv.hpp>

#include "boundingbox.hpp"

namespace rmcv::detect
{
    struct LightBar
    {
        /*
         * width始终为长边，angle始终为长边从 向正右方射线 沿逆时针转动的角度即[0, 180)
         */
        cv::Point2f center;
        float width, height, rad;
    };

    LightBar minAreaRect(cv::InputArray points);

    BoundingBox fix_boundingbox(const BoundingBox &ori_bbox, const cv::Mat &ori_img);
}

#endif