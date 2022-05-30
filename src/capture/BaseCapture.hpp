#ifndef __CAPTURE_BASE_HPP__
#define __CAPTURE_BASE_HPP__

#include <opencv2/opencv.hpp>

namespace rmcv::capture
{
    // 使用 std::string 作为路径的数据类型
    using Path = std::string;

    // 所有捕获器的基类
    class BaseCapture
    {
    public:
        /**
         * @brief 捕获下一帧图像
         * 
         * @return cv::Mat 图像
         */
        virtual cv::Mat next() = 0;
    };

    // 相机捕获基类
    class BaseCameraCapture: public BaseCapture
    {
    public:
        /**
         * @brief 设置曝光时间
         *
         * @param time 时间
         * @return true 成功
         * @return false 失败
         */
        virtual bool set_exposure_time(float time) = 0;

        /**
         * @brief 设置增益
         *
         * @param gain 增益
         * @return true 成功
         * @return false 失败
         */
        virtual bool set_gain(float gain) = 0;

        /**
         * @brief 设置白平衡
         *
         * @param red 红色通道
         * @param green 绿色通道
         * @param blue 蓝色通道
         * @return true 成功
         * @return false 失败
         */
        virtual bool set_white_balance(float red, float green, float blue) = 0;
    };
}

#endif