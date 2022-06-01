#ifndef __CAPTURE_DAHUA_CAPTURE_HPP__
#define __CAPTURE_DAHUA_CAPTURE_HPP__

#include <memory>
#include <opencv2/opencv.hpp>

#include "base_capture.hpp"

namespace rmcv::capture
{
class DahuaCapture : public BaseCameraCapture
{
 private:
  class Impl;
  std::unique_ptr<Impl> pimpl;

 public:
  DahuaCapture();

  ~DahuaCapture();

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