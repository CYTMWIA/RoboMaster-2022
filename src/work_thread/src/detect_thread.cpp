#include "work_thread/detect_thread.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>

#include "common/logging.hpp"
#include "common/threading.hpp"
#include "communicate/communicate.hpp"
#include "detect/detect.hpp"

namespace rm_work_thread
{
DetectThread::DetectThread(const rm_config::Config &cfg)
    : armor_detector(rm_detect::OcvArmorDetectorSettings{cfg.model.path})
{
}

void DetectThread::run()
{
  using namespace rm_threading;
  using namespace rm_detect;
  using namespace rm_communicate;
  // auto fps = rm_util::FpsCounter();
  while (true)
  {
    // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});
    auto robot_status = RoslikeTopic<RobotStatus>::get("robot_status", true);
    auto img = RoslikeTopic<cv::Mat>::get("capture_image");
    if (robot_status.target >= 2)
    {
      // auto res = power_rune_detector(img);
      // RoslikeTopic<decltype(res)>::set("detect_result", std::move(res));
    }
    else
    {
      auto res = armor_detector(img);
      RoslikeTopic<decltype(res)>::set("detect_result", std::move(res));
    }
  }
}
}  // namespace rm_work_thread