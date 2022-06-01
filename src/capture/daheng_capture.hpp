#ifndef __CAPTURE_DAHENG_CAPTURE_HPP__
#define __CAPTURE_DAHENG_CAPTURE_HPP__

#include <opencv2/opencv.hpp>

#include "base_capture.hpp"
#include "daheng/GxIAPI.h"

namespace rmcv::capture
{
class DahengCapture : public BaseCameraCapture
{
 private:
  GX_DEV_HANDLE dev_;

  /**
   * @brief 获取设备数量
   *
   * @return int
   */
  uint32_t get_devices_count();

 public:
  /**
   * @brief DahengCapture 构造函数，会初始化大恒库及打开相机
   *
   * @param device_id 设备编号
   */
  DahengCapture(int device_id = 1);

  /**
   * @brief DahengCapture 析构函数，会关闭相机及关闭大恒库
   *
   */
  ~DahengCapture();

  /**
   * @brief 设置曝光时间
   *
   * @param time 时间
   * @return true 成功
   * @return false 失败
   */
  bool set_exposure_time(float time) override;

  /**
   * @brief 设置增益
   *
   * @param gain 增益
   * @return true 成功
   * @return false 失败
   */
  bool set_gain(float gain) override;

  /**
   * @brief 设置白平衡
   *
   * @param red 红色通道
   * @param green 绿色通道
   * @param blue 蓝色通道
   * @return true 成功
   * @return false 失败
   */
  bool set_white_balance(float red, float green, float blue) override;

  cv::Mat next() override;
};
}  // namespace rmcv::capture

#endif