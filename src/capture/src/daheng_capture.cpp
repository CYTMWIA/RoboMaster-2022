#include "capture/daheng_capture.hpp"

#include <opencv2/opencv.hpp>

#include "common/logging.hpp"
#include "daheng/DxImageProc.h"

namespace rm_capture
{
DahengCapture::DahengCapture(int device_id)
{
  if (GXInitLib() != GX_STATUS_SUCCESS) __LOG_ERROR_AND_EXIT("初始化库失败");

  if (get_devices_count() <= 0) __LOG_ERROR_AND_EXIT("未发现设备");

  if (GXOpenDeviceByIndex(device_id, &dev_) != GX_STATUS_SUCCESS)
    __LOG_ERROR_AND_EXIT("打开设备失败");

  if (GXStreamOn(dev_) != GX_STATUS_SUCCESS) __LOG_ERROR_AND_EXIT("开采失败");
}

uint32_t DahengCapture::get_devices_count()
{
  uint32_t devices;
  if (GXUpdateDeviceList(&devices, 500) != GX_STATUS_SUCCESS)
    __LOG_ERROR_AND_EXIT("获取设备数量失败");
  return devices;
}

bool DahengCapture::set_exposure_time(float time)
{
  int err = 0;
  err += GXSetEnum(dev_, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
  err += GXSetEnum(dev_, GX_ENUM_EXPOSURE_TIME_MODE, GX_EXPOSURE_TIME_MODE_STANDARD);
  err += GXSetEnum(dev_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
  err += GXSetFloat(dev_, GX_FLOAT_EXPOSURE_TIME, time);

  if (err)
  {
    __LOG_WARNING("曝光时间设置失败");
    return false;
  }
  else
  {
    return true;
  }
}

bool DahengCapture::set_gain(float gain)
{
  if (GXSetFloat(dev_, GX_FLOAT_GAIN, gain) == GX_STATUS_SUCCESS)
    return true;
  else
  {
    __LOG_WARNING("增益设置失败");
    return false;
  }
}

bool DahengCapture::set_white_balance(float red, float green, float blue)
{
  int err = 0;
  err += GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
  err += GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, red);
  err += GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
  err += GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, green);
  err += GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
  err += GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, blue);

  if (err)
  {
    __LOG_WARNING("白平衡设置失败\n");
    return false;
  }
  else
  {
    return true;
  }
}

cv::Mat DahengCapture::next()
{
  PGX_FRAME_BUFFER buf;
  cv::Mat img;

  if (GXDQBuf(dev_, &buf, 500) != GX_STATUS_SUCCESS)
  {
    throw std::runtime_error("获取图像buf失败");
  }

  img.create(buf->nHeight, buf->nWidth, CV_8UC3);
  DxRaw8toRGB24(buf->pImgBuf, img.data, buf->nWidth, buf->nHeight, RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(BAYERBG), false);

  if (GXQBuf(dev_, buf) != GX_STATUS_SUCCESS)
  {
    __LOG_ERROR_AND_EXIT("放回图像buf失败");
  }

  return img;
}

DahengCapture::~DahengCapture()
{
  GXCloseDevice(dev_);
  GXCloseLib();
}
}  // namespace rm_capture