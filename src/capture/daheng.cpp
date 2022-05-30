#include <opencv2/opencv.hpp>

#include "logging.hpp"

#include "daheng/DxImageProc.h"

#include "daheng.hpp"

namespace rmcv::capture
{
    DahengCapturer::DahengCapturer(int device_id)
    {
        if (GXInitLib() != GX_STATUS_SUCCESS)
            __LOG_ERROR_AND_EXIT("初始化库失败");
        
        if (get_devices_count() <= 0)
            __LOG_ERROR_AND_EXIT("未发现设备");
        
        if (GXOpenDeviceByIndex(device_id, &dev_) != GX_STATUS_SUCCESS)
            __LOG_ERROR_AND_EXIT("打开设备失败");

        if (GXStreamOn(dev_) != GX_STATUS_SUCCESS)
            __LOG_ERROR_AND_EXIT("开采失败");
    }

    uint32_t DahengCapturer::get_devices_count()
    {
        uint32_t devices;
        if (GXUpdateDeviceList(&devices, 500) != GX_STATUS_SUCCESS)
            __LOG_ERROR_AND_EXIT("获取设备数量失败");
        return devices;
    }

    bool DahengCapturer::set_exposure_time(float time)
    {
        if (GXSetFloat(dev_, GX_FLOAT_EXPOSURE_TIME, time) == GX_STATUS_SUCCESS)
            return true;
        else
        {
            __LOG_WARNING("曝光时间设置失败");
            return false;
        }
    }

    bool DahengCapturer::set_gain(float gain)
    {
        if (GXSetFloat(dev_, GX_FLOAT_GAIN, gain) == GX_STATUS_SUCCESS)
            return true;
        else
        {
            __LOG_WARNING("增益设置失败");
            return false;
        }
    }

    bool DahengCapturer::set_white_balance(float red, float green, float blue)
    {
        if (   GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED) == GX_STATUS_SUCCESS
            && GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, red) == GX_STATUS_SUCCESS
            && GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN) == GX_STATUS_SUCCESS
            && GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, green) == GX_STATUS_SUCCESS     
            && GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE) == GX_STATUS_SUCCESS
            && GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, blue) == GX_STATUS_SUCCESS)
        {
            return true;
        }
        else
        {
            __LOG_WARNING("白平衡设置失败\n");
            return false;
        }
    }

    cv::Mat DahengCapturer::next()
    {
        PGX_FRAME_BUFFER buf;
        cv::Mat img;

        if (GXDQBuf(dev_, &buf, 500) != GX_STATUS_SUCCESS)
        {
            throw std::runtime_error("获取图像buf失败");
        }

        img.create(buf->nHeight, buf->nWidth, CV_8UC3);
        DxRaw8toRGB24(buf->pImgBuf, img.data, buf->nWidth, buf->nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);

        if (GXQBuf(dev_, buf) != GX_STATUS_SUCCESS)
        {
            __LOG_ERROR_AND_EXIT("放回图像buf失败");
        }

        return img;
    }

    DahengCapturer::~DahengCapturer()
    {
        GXCloseDevice(dev_);
        GXCloseLib();
    }
}