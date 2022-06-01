#include "work_thread/capture_thread.hpp"

#include <opencv2/opencv.hpp>

#include "capture/capture.hpp"
#include "common/logging.hpp"
#include "common/threading.hpp"

namespace rm_work_thread
{
CaptureThread::CaptureThread(const rm_config::Config &cfg)
{
  target_ = cfg.capture.target;

  if (target_ == "image")
  {
    target_func_ = std::bind(&CaptureThread::image, this, cfg.image.path);
  }
  else if (target_ == "video")
  {
    target_func_ = std::bind(&CaptureThread::video, this, cfg.video.path);
  }
  else if (target_ == "camera")
  {
    target_func_ = std::bind(&CaptureThread::camera, this);
    camera_ = cfg.camera;
  }
  else
  {
    __LOG_ERROR_AND_EXIT("未知捕获目标");
  }
}

void CaptureThread::image(const std::string path)
{
  using namespace rm_threading;

  cv::Mat img = cv::imread(path);

  RoslikeTopic<cv::Mat>::set("capture_image", img);
}

void CaptureThread::video(const std::string path)
{
  using namespace rm_threading;

  auto cp = rm_capture::VideoCapture(path);

  while (true)
  {
    rm_threading::RoslikeTopic<cv::Mat>::set("capture_image", cp.next());
  }
}

void CaptureThread::init_camera(std::unique_ptr<rm_capture::BaseCameraCapture> &pcap)
{
  using namespace rm_capture;

  if (camera_.manufacturer == "dahua")
  {
    pcap = std::make_unique<DahuaCapture>();
  }
  else if (camera_.manufacturer == "daheng")
  {
    pcap = std::make_unique<DahengCapture>();
  }
  else
  {
    __LOG_ERROR_AND_EXIT("未指定相机厂商");
  }

  pcap->set_exposure_time(camera_.exposure_time);
  pcap->set_gain(camera_.gain);
  pcap->set_white_balance(camera_.white_balance_red, camera_.white_balance_green,
                          camera_.white_balance_blue);
}

void CaptureThread::camera()
{
  using namespace rm_threading;
  using namespace rm_capture;

  std::unique_ptr<BaseCameraCapture> pcap;
  init_camera(pcap);

  // auto fps = rm_util::FpsCounter();
  while (true)
  {
    // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});
    try
    {
      rm_threading::RoslikeTopic<cv::Mat>::set("capture_image", pcap->next());
    }
    catch (std::runtime_error e)
    {
      __LOG_ERROR("{}", e.what());
      init_camera(pcap);
    }
  }
}

void CaptureThread::run(void) { target_func_(); }
}  // namespace rm_work_thread