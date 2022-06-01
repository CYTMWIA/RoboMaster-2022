#ifndef __WORK_THREAD_CAPTURE_THREAD_HPP__
#define __WORK_THREAD_CAPTURE_THREAD_HPP__

/*
 * 捕获到的图像名称为 "capture_image"
 */

#include <functional>
#include <thread>

#include "base_thread.hpp"
#include "capture/capture.hpp"
#include "config/config.hpp"

namespace rm_work_thread
{
DECL_WORKTHTREAD(CaptureThread)

private:
std::string target_;
std::function<void(void)> target_func_;

rm_config::Config::Camera camera_;

void image(const std::string path);
void video(const std::string path);
void init_camera(std::unique_ptr<rm_capture::BaseCameraCapture> &pcap);
void camera();

public:
CaptureThread(const rm_config::Config &cfg);

DECL_WORKTHTREAD_END()
}  // namespace rm_work_thread

#endif