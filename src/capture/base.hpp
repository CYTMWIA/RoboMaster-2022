#ifndef __BASE_HPP__
#define __BASE_HPP__

#include <opencv2/opencv.hpp>

namespace rmcv::capture
{
    // 使用 std::string 作为路径的数据类型
    using Path = std::string;

    // 所有捕获器的基类
    struct BaseCapturer
    {
        /**
         * @brief 捕获下一帧图像
         * 
         * @return cv::Mat 图像
         */
        virtual cv::Mat next() = 0;
    };
}

#endif