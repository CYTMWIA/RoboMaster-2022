#include "rm_capture/camera_node.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include "base_camera.hpp"
#include "daheng_camera.hpp"
#include "dahua_camera.hpp"
#include "nerv/nerv.hpp"

namespace rm_capture
{
struct CameraNode::Impl
{
  // 参数
  std::string manufacturer;
  double exposure_time;
  double gain;
  double white_balance_red;
  double white_balance_green;
  double white_balance_blue;
  // 成员变量
  std::unique_ptr<BaseCamera> pcam_;
  void set_camera_params()
  {
    pcam_->set_exposure_time(exposure_time);
    pcam_->set_gain(gain);
    pcam_->set_white_balance(white_balance_red, white_balance_green, white_balance_blue);
  }
  void reopen_camera()
  {
    while (true)
    {
      NERV_WARN("尝试重新打开相机");
      try
      {
        pcam_->reopen();
        set_camera_params();
        break;
      }
      catch (const std::exception &ex)
      {
        NERV_ERROR("重新打开相机失败：{}", ex.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
    NERV_INFO("重新打开相机成功");
  }
  void open_camera()
  {
    try
    {
      pcam_->open();
      set_camera_params();
    }
    catch (const std::exception &ex)
    {
      NERV_ERROR("打开相机失败：{}", ex.what());
      reopen_camera();
    }
  }
};

CameraNode::~CameraNode() = default;

CameraNode::CameraNode() : nerv::Node("camera_node"), pimpl_(new Impl())
{
  bool invalid_argument = false;
  pimpl_->manufacturer = this->get_parameter<std::string>("manufacturer", "");
  if (pimpl_->manufacturer == "dahua")
    pimpl_->pcam_ = std::make_unique<DahuaCamera>();
  else if (pimpl_->manufacturer == "daheng")
    pimpl_->pcam_ = std::make_unique<DahengCamera>();
  else
  {
    NERV_ERROR("参数错误 manufacturer");
    invalid_argument = true;
  }
  pimpl_->exposure_time = this->get_parameter<double>("exposure_time", 5000);
  pimpl_->gain = this->get_parameter<double>("gain", 1);
  pimpl_->white_balance_red = this->get_parameter<double>("white_balance_red", 1);
  pimpl_->white_balance_green = this->get_parameter<double>("white_balance_green", 1);
  pimpl_->white_balance_blue = this->get_parameter<double>("white_balance_blue", 1);
  if (invalid_argument) throw std::invalid_argument("");
}

void CameraNode::run()
{
  pimpl_->open_camera();
  while (true)
  {
    cv::Mat frame;

    try
    {
      frame = pimpl_->pcam_->next();
    }
    catch (const std::exception &ex)
    {
      NERV_ERROR("获取帧失败：{}", ex.what());
      pimpl_->reopen_camera();
      continue;
    }

    nerv::Topic<cv::Mat>::set("capture_frame", std::move(frame));
  }
}

}  // namespace rm_capture