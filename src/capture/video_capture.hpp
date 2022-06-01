#ifndef __CAPTURE_VIDEO_CAPTURE_HPP__
#define __CAPTURE_VIDEO_CAPTURE_HPP__

#include <chrono>
#include <string>

#include "base_capture.hpp"

namespace rmcv::capture
{
class VideoCapture : public BaseCapture
{
 private:
  Path path_;
  bool loop_;
  cv::VideoCapture cap_;
  std::chrono::milliseconds frame_interval_;
  std::chrono::time_point<std::chrono::steady_clock> next_frame_time_ =
      std::chrono::steady_clock::now();

 public:
  VideoCapture() = delete;

  /**
   * @brief VideoCapture 构造函数
   *
   * @param path 视频路径
   * @param loop 是否循环播放
   */
  VideoCapture(Path path, bool loop = true);

  cv::Mat next() override;
};
}  // namespace rmcv::capture

#endif