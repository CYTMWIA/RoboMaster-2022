#include "CaptureThread.hpp"

#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "capture/capture.hpp"

#include <opencv2/opencv.hpp>

namespace rmcv::work_thread
{
    CaptureThread::CaptureThread(const rmcv::config::Config &cfg)
    {
        target_ = cfg.capture.target;

        if (target_ == "image")
        {
            target_func_ = std::bind(&CaptureThread::image, this, cfg.image.path);
        }
        else if (target_ == "video")
        {
            __LOG_ERROR_AND_EXIT("捕获视频尚未实现（没需求+懒");
        }
        else if (target_ == "camera")
        {
            target_func_ = std::bind(&CaptureThread::camera, this, cfg.camera.id, cfg.camera.exposure_time, cfg.camera.gain, cfg.camera.white_balance_red, cfg.camera.white_balance_green, cfg.camera.white_balance_blue);
        }
        else
        {
            __LOG_ERROR_AND_EXIT("未知捕获目标");
        }
    }

    void CaptureThread::image(const std::string path)
    {
        using namespace rmcv::threading;

        cv::Mat img = cv::imread(path);

        RoslikeTopic<cv::Mat>::set("capture_image", img);
    }

    void CaptureThread::camera(int32_t id, double exposure_time, double gain, double white_balance_red, double white_balance_green, double white_balance_blue)
    {
        using namespace rmcv::threading;

        auto cp = rmcv::capture::DahengCapturer(id);
        cp.set_exposure_time(exposure_time);
        cp.set_gain(gain);
        cp.set_white_balance(white_balance_red, white_balance_green, white_balance_blue);

        while (true)
        {
            threading::RoslikeTopic<cv::Mat>::set("capture_image", cp.next());
        }
    }

    void CaptureThread::up()
    {
        thread_ = std::thread(target_func_);
    }
}