#ifndef __CAPTURE_IMAGE_CAPTURE_HPP__
#define __CAPTURE_IMAGE_CAPTURE_HPP__

#include "base_capture.hpp"

namespace rmcv::capture
{
class ImageCapture : public BaseCapture
{
 private:
  Path path_;

 public:
  ImageCapture() = delete;

  /**
   * @brief ImageCapture 构造函数
   *
   * @param path 路径
   */
  ImageCapture(Path path);

  cv::Mat next() override;
};
}  // namespace rmcv::capture

#endif