#include "rm_capture/video_capture_node.hpp"

#include <chrono>
#include <memory>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <thread>

#include "nerv/logging.hpp"
#include "nerv/nerv.hpp"
#include "nerv/timer.hpp"

namespace rm_capture
{
struct VideoCaptureNode::Impl
{
  std::string capture_target;
  int camera_index;
  std::string file_path;

  cv::VideoCapture cap_;
  void open_capture()
  {
    int ok;
    if (capture_target == "file")
      ok = cap_.open(file_path);
    else
      ok = cap_.open(camera_index);
    if (ok) NERV_INFO("打开 cv::VideoCapture 成功");
  }
};
VideoCaptureNode::~VideoCaptureNode() = default;

VideoCaptureNode::VideoCaptureNode() : nerv::Node("video_capture_node"), pimpl_(new Impl())
{
  bool invalid_argument = false;
  pimpl_->capture_target = this->get_parameter<std::string>("capture_target", "");
  if (pimpl_->capture_target == "file")
  {
    pimpl_->file_path = this->get_parameter<std::string>("file_path", "");
    if (pimpl_->file_path == "")
    {
      NERV_ERROR("缺少参数 file_path");
      invalid_argument = true;
    }
  }
  else if (pimpl_->capture_target == "camera")
  {
    pimpl_->camera_index = this->get_parameter<int>("camera_index", 0);
  }
  else
  {
    NERV_ERROR("参数错误 capture_target");
    invalid_argument = true;
  }
  if (invalid_argument) throw std::invalid_argument("");
}

void VideoCaptureNode::run()
{
  pimpl_->open_capture();
  while (true)
  {
    cv::Mat frame;

    {
      nerv::ScopedTimer tim("capture");
      if (pimpl_->cap_.isOpened()) pimpl_->cap_.read(frame);
    }

    if (frame.empty())
    {
      // 重新打开
      NERV_WARN("尝试重新打开 cv::VideoCapture");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      pimpl_->open_capture();
      continue;
    }

    {
      nerv::ScopedTimer tim("topic set");
      nerv::Topic<cv::Mat>::set("capture_frame", std::move(frame));
    }
  }
}

}  // namespace rm_capture