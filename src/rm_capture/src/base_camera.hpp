#ifndef __BASE_CAMERA_HPP__
#define __BASE_CAMERA_HPP__

#include <opencv2/opencv.hpp>

namespace rm_capture
{

class BaseCamera
{
 public:
  /**
   * @brief 打开相机
   *
   * @param index 相机编号
   */
  virtual void open(int index = -1) = 0;

  /**
   * @brief 重新打开相机
   *
   * @param index 相机编号
   */
  virtual void reopen(int index = -1) = 0;

  /**
   * @brief 捕获下一帧图像
   *
   * @return cv::Mat 图像
   */
  virtual cv::Mat next() = 0;

  /**
   * @brief 设置曝光时间
   *
   * @param time 时间
   */
  virtual void set_exposure_time(double time) = 0;

  /**
   * @brief 设置增益
   *
   * @param gain 增益
   */
  virtual void set_gain(double gain) = 0;

  /**
   * @brief 设置白平衡
   *
   * @param red 红色通道
   * @param green 绿色通道
   * @param blue 蓝色通道
   */
  virtual void set_white_balance(double red, double green, double blue) = 0;
};

}  // namespace rm_capture

#endif