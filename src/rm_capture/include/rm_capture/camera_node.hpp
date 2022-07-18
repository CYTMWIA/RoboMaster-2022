#ifndef __RM_CAPTURE_CAMERA_NODE_HPP__
#define __RM_CAPTURE_CAMERA_NODE_HPP__

#include <memory>

#include "nerv/nerv.hpp"

namespace rm_capture
{

class CameraNode : public nerv::Node
{
 private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;

 public:
  CameraNode();
  ~CameraNode();

  void run() override;
};

}  // namespace rm_capture

#endif