#include "rm_node/detect_thread.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>

#include "rm_common/logging.hpp"
#include "rm_common/threading.hpp"
#include "rm_detect/detect.hpp"

namespace rm_node
{
DetectThread::DetectThread(const rm_config::Config &cfg)
    :  // detector(cfg.model.path)
      detector()
{
}

void DetectThread::run()
{
  using namespace rm_threading;
  using namespace rm_detect;

  // auto fps = rm_util::FpsCounter();
  while (true)
  {
    // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});

    auto img = RoslikeTopic<cv::Mat>::get("capture_image");
    auto res = detector(img);
    RoslikeTopic<decltype(res)>::set("detect_result", std::move(res));
  }
}
}  // namespace rm_node