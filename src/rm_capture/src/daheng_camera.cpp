#include "daheng_camera.hpp"

#include <stdexcept>

#include "daheng/GxIAPI.h"
#include "nerv/logging.hpp"

namespace rm_capture
{
std::string DahengCamera::error_code_to_string(int code)
{
  switch (code)
  {
    case GX_STATUS_SUCCESS:
      return std::string("Success");
    case GX_STATUS_ERROR:
      return std::string("There is an unspecified internal error that is not expected to occur");
    case GX_STATUS_NOT_FOUND_TL:
      return std::string("The TL library cannot be found");
    case GX_STATUS_NOT_FOUND_DEVICE:
      return std::string("The device is not found");
    case GX_STATUS_OFFLINE:
      return std::string("The current device is in an offline status");
    case GX_STATUS_INVALID_PARAMETER:
      return std::string(
          "Invalid parameter. Generally, the pointer is NULL or the input IP and other parameter "
          "formats are invalid");
    case GX_STATUS_INVALID_HANDLE:
      return std::string("Invalid handle");
    case GX_STATUS_INVALID_CALL:
      return std::string(
          "The interface is invalid, which refers to software interface logic error");
    case GX_STATUS_INVALID_ACCESS:
      return std::string(
          "The function is currently inaccessible or the device access mode is incorrect");
    case GX_STATUS_NEED_MORE_BUFFER:
      return std::string(
          "The user request buffer is insufficient: the user input buffer size during the read "
          "operation is less than the actual need");
    case GX_STATUS_ERROR_TYPE:
      return std::string(
          "The type of FeatureID used by the user is incorrect, such as an integer interface using "
          "a floating-point function code");
    case GX_STATUS_OUT_OF_RANGE:
      return std::string("The value written by the user is crossed");
    case GX_STATUS_NOT_IMPLEMENTED:
      return std::string("This function is not currently supported");
    case GX_STATUS_NOT_INIT_API:
      return std::string("There is no call to initialize the interface");
    case GX_STATUS_TIMEOUT:
      return std::string("Timeout error");
  }
  return std::string("未知错误");
}

void DahengCamera::open(int index)
{
  if (index < 1) index = 1;
  int err;

  err = GXInitLib();
  if (err != GX_STATUS_SUCCESS)
    throw std::runtime_error("初始化大恒库失败：" + error_code_to_string(err));

  uint32_t devices;
  err = GXUpdateDeviceList(&devices, 500);
  if (err != GX_STATUS_SUCCESS)
    throw std::runtime_error("获取设备数量失败：" + error_code_to_string(err));
  if (devices <= 0) throw std::runtime_error("未发现设备");

  err = GXOpenDeviceByIndex(index, &dev_);
  if (err != GX_STATUS_SUCCESS)
    throw std::runtime_error("打开相机失败：" + error_code_to_string(err));

  err = GXStreamOn(dev_);
  if (err != GX_STATUS_SUCCESS) throw std::runtime_error("拉流失败：" + error_code_to_string(err));
}

void DahengCamera::reopen(int index)
{
  int err;
  err = GXCloseDevice(dev_);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("关闭相机失败：{}", error_code_to_string(err));
  err = GXCloseLib();
  if (err != GX_STATUS_SUCCESS) NERV_WARN("关闭大恒库失败：{}", error_code_to_string(err));
  open(index);
}

cv::Mat DahengCamera::next()
{
  int err;
  PGX_FRAME_BUFFER buf;
  cv::Mat frame;

  err = GXDQBuf(dev_, &buf, 500);
  if (err != GX_STATUS_SUCCESS)
    throw std::runtime_error("获取图像buf失败：" + error_code_to_string(err));

  frame.create(buf->nHeight, buf->nWidth, CV_8UC3);
  // 注意：没有处理这里的返回值（但问题应该不大，因为图像始终正常）
  DxRaw8toRGB24(buf->pImgBuf, frame.data, buf->nWidth, buf->nHeight, RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(BAYERBG), false);

  err = GXQBuf(dev_, buf);
  if (err != GX_STATUS_SUCCESS)
    throw std::runtime_error("放回图像buf失败：" + error_code_to_string(err));

  return frame;
}

void DahengCamera::set_exposure_time(double time)
{
  int err;
  err = GXSetEnum(dev_, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("曝光时间设置失败：{}", error_code_to_string(err));
  err = GXSetEnum(dev_, GX_ENUM_EXPOSURE_TIME_MODE, GX_EXPOSURE_TIME_MODE_STANDARD);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("曝光时间设置失败：{}", error_code_to_string(err));
  err = GXSetEnum(dev_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("曝光时间设置失败：{}", error_code_to_string(err));
  err = GXSetFloat(dev_, GX_FLOAT_EXPOSURE_TIME, time);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("曝光时间设置失败：{}", error_code_to_string(err));
}

void DahengCamera::set_gain(double gain)
{
  int err = GXSetFloat(dev_, GX_FLOAT_GAIN, gain);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("增益设置失败：{}", error_code_to_string(err));
}

void DahengCamera::set_white_balance(double red, double green, double blue)
{
  int err;

  err = GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("白平衡设置失败：{}", error_code_to_string(err));
  err = GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, red);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("白平衡设置失败：{}", error_code_to_string(err));

  err = GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("白平衡设置失败：{}", error_code_to_string(err));
  err = GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, green);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("白平衡设置失败：{}", error_code_to_string(err));

  err = GXSetEnum(dev_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("白平衡设置失败：{}", error_code_to_string(err));
  err = GXSetFloat(dev_, GX_FLOAT_BALANCE_RATIO, blue);
  if (err != GX_STATUS_SUCCESS) NERV_WARN("白平衡设置失败：{}", error_code_to_string(err));
}

}  // namespace rm_capture