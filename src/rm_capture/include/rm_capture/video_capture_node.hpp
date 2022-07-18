#ifndef __RM_CAPTURE_VIDEO_CAPTURE_NODE_HPP__
#define __RM_CAPTURE_VIDEO_CAPTURE_NODE_HPP__

#include <memory>

#include "nerv/nerv.hpp"

namespace rm_capture
{

class VideoCaptureNode : public nerv::Node
{
 private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;

 public:
  VideoCaptureNode();
  ~VideoCaptureNode();

  void run() override;
};

}  // namespace rm_capture

#endif