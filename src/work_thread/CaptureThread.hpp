#ifndef __WORK_THREAD_CAPTURETHREAD_HPP__
#define __WORK_THREAD_CAPTURETHREAD_HPP__

/*
 * 捕获到的图像名称为 "capture_image"
 */

#include "BaseThread.hpp"

#include "config/config.hpp"
#include "capture/capture.hpp"

#include <thread>
#include <functional>

namespace rmcv::work_thread
{
    DECL_WORKTHTREAD(CaptureThread)

private:
    std::string target_;
    std::function<void(void)> target_func_;

    rmcv::config::Config::Camera camera_;

    void image(const std::string path);
    void video(const std::string path);
    void init_camera(std::unique_ptr<rmcv::capture::BaseCameraCapture> &pcap);
    void camera();

public:
    CaptureThread(const rmcv::config::Config &cfg);

    DECL_WORKTHTREAD_END()
}

#endif