#include "dahua_camera.hpp"

#include "nerv/logging.hpp"

namespace rm_capture
{
std::string DahuaCamera::error_code_to_string(int code)
{
  switch (code)
  {
    case IMV_OK:
      return std::string("成功，无错误");
    case IMV_ERROR:
      return std::string("通用的错误");
    case IMV_INVALID_HANDLE:
      return std::string("错误或无效的句柄");
    case IMV_INVALID_PARAM:
      return std::string("错误的参数");
    case IMV_INVALID_FRAME_HANDLE:
      return std::string("错误或无效的帧句柄");
    case IMV_INVALID_FRAME:
      return std::string("无效的帧");
    case IMV_INVALID_RESOURCE:
      return std::string("相机/事件/流等资源无效");
    case IMV_INVALID_IP:
      return std::string("设备与主机的IP网段不匹配");
    case IMV_NO_MEMORY:
      return std::string("内存不足");
    case IMV_INSUFFICIENT_MEMORY:
      return std::string("传入的内存空间不足");
    case IMV_ERROR_PROPERTY_TYPE:
      return std::string("属性类型错误");
    case IMV_INVALID_ACCESS:
      return std::string("属性不可访问、或不能读/写、或读/写失败");
    case IMV_INVALID_RANGE:
      return std::string("属性值超出范围、或者不是步长整数倍");
    case IMV_NOT_SUPPORT:
      return std::string("设备不支持的功能");
  }
  return std::string("未知错误");
}

void DahuaCamera::open(int index)
{
  if (index < 0) index = 0;

  int err = IMV_OK;

  IMV_DeviceList devices;
  err = IMV_EnumDevices(&devices, interfaceTypeAll);
  if (err != IMV_OK) throw std::runtime_error("获取设备列表失败：" + error_code_to_string(err));
  if (devices.nDevNum < 1) throw std::runtime_error("未发现设备");

  err = IMV_CreateHandle(&dev_, modeByIndex, (void *)&index);
  if (err != IMV_OK) throw std::runtime_error("创建设备句柄失败：" + error_code_to_string(err));
  err = IMV_Open(dev_);
  if (err != IMV_OK) throw std::runtime_error("打开相机失败：" + error_code_to_string(err));

  err = IMV_StartGrabbing(dev_);
  if (err != IMV_OK) throw std::runtime_error("拉流失败：" + error_code_to_string(err));
}

void DahuaCamera::reopen(int index)
{
  int err = IMV_DestroyHandle(dev_);
  if (err != IMV_OK) NERV_WARN("摧毁设备句柄失败：{}", error_code_to_string(err));
  open(index);
}

cv::Mat DahuaCamera::next()
{
  int err;
  IMV_Frame frame;
  cv::Mat img;

  err = IMV_GetFrame(dev_, &frame, 500);
  if (err != IMV_OK) throw std::runtime_error("获取图像失败：" + error_code_to_string(err));

  img.create(frame.frameInfo.height, frame.frameInfo.width, CV_8UC3);
  IMV_PixelConvertParam param;
  param.nWidth = frame.frameInfo.width;
  param.nHeight = frame.frameInfo.height;
  param.ePixelFormat = frame.frameInfo.pixelFormat;
  param.pSrcData = frame.pData;
  param.nSrcDataLen = frame.frameInfo.size;
  param.nPaddingX = frame.frameInfo.paddingX;
  param.nPaddingY = frame.frameInfo.paddingY;
  param.eBayerDemosaic = demosaicNearestNeighbor;
  param.eDstPixelFormat = gvspPixelBGR8;
  param.pDstBuf = img.data;
  param.nDstBufSize = param.nWidth * param.nHeight * 3;
  param.nDstDataLen = param.nWidth * param.nHeight * 3;

  err = IMV_PixelConvert(dev_, &param);
  if (err != IMV_OK) throw std::runtime_error("转换像素失败：" + error_code_to_string(err));

  err = IMV_ReleaseFrame(dev_, &frame);
  if (err != IMV_OK) throw std::runtime_error("释放图像缓存失败：" + error_code_to_string(err));

  return img;
}

#define SET_IMV(type, name, value)                                   \
  err = IMV_Set##type##FeatureValue(dev_, (name), (value));          \
  if (err != IMV_OK)                                                 \
  {                                                                  \
    NERV_WARN("设置 " #name " 失败：%s", error_code_to_string(err)); \
  }

void DahuaCamera::set_exposure_time(double value)
{
  int err;
  SET_IMV(Double, "ExposureTime", value);
}

void DahuaCamera::set_gain(double value)
{
  int err;
  SET_IMV(Double, "GainRaw", value);
}

void DahuaCamera::set_white_balance(double red, double green, double blue)
{
#define SET_IMV_WHITE_BALANCE(ch, val)        \
  SET_IMV(Enum, "BalanceWhiteAuto", 0)        \
  SET_IMV(Enum, "BalanceRatioSelector", (ch)) \
  SET_IMV(Double, "BalanceRatio", (val))

  int err;
  SET_IMV_WHITE_BALANCE(0, red);
  SET_IMV_WHITE_BALANCE(1, green);
  SET_IMV_WHITE_BALANCE(2, blue);

#undef SET_IMV_WHITE_BALANCE
}
#undef SET_IMV

}  // namespace rm_capture