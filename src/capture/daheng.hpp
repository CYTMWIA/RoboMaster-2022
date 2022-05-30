#ifndef __DAHENG_HPP__
#define __DAHENG_HPP__

#include <opencv2/opencv.hpp>

#include "daheng/GxIAPI.h"

#include "base.hpp"

namespace rmcv::capture
{
    class DahengCapturer: public BaseCapturer
    {
    private:
        GX_DEV_HANDLE dev_;
    public:
        DahengCapturer() = delete;

        /**
         * @brief DahengCapturer 构造函数，会初始化大恒库及打开相机
         * 
         * @param device_id 设备编号
         */
        DahengCapturer(int device_id);

        /**
         * @brief DahengCapturer 析构函数，会关闭相机及关闭大恒库
         * 
         */
        ~DahengCapturer();

        /**
         * @brief 获取设备数量
         * 
         * @return int 
         */
        uint32_t get_devices_count();

        /**
         * @brief 设置曝光时间
         * 
         * @param time 时间
         * @return true 成功
         * @return false 失败
         */
        bool set_exposure_time(float time);

        /**
         * @brief 设置增益
         * 
         * @param gain 增益
         * @return true 成功
         * @return false 失败
         */
        bool set_gain(float gain);

        /**
         * @brief 设置白平衡
         * 
         * @param red 红色通道
         * @param green 绿色通道
         * @param blue 蓝色通道
         * @return true 成功
         * @return false 失败
         */
        bool set_white_balance(float red, float green, float blue);

        cv::Mat next();
    private:

    };    
}

#endif