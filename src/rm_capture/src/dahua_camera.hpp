#ifndef __DAHUA_CAMERA_HPP__
#define __DAHUA_CAMERA_HPP__

#include "base_camera.hpp"

// 大华
#include <IMVApi.h>
#include <IMVDefines.h>

namespace rm_capture
{
class DahuaCamera : public BaseCamera
{
 private:
  IMV_HANDLE dev_;
  std::string error_code_to_string(int code);

 public:
  void open(int index = -1) override;
  void reopen(int index = -1) override;

  cv::Mat next() override;

  void set_exposure_time(double time) override;
  void set_gain(double gain) override;
  void set_white_balance(double red, double green, double blue) override;
};
}  // namespace rm_capture

#endif