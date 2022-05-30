#include <opencv2/opencv.hpp>

#include "capture/capture.hpp"
#include "logging/logging.hpp"
#include "threading/threading.hpp"

#include "capture.hpp"

namespace rmcv
{
    using namespace config;
    using namespace threading;

    void capture_image(const Config &cfg)
    {
        cv::Mat img = cv::imread(cfg.image.path);

        RoslikeTopic<cv::Mat>::set("capture_image", img);
    }

    void capture_video(const Config &cfg)
    {
        // TODO
        __LOG_ERROR_AND_EXIT("未实现的函数！");
    }

    void capture_camera(const Config &cfg)
    {
        auto cp = capture::DahengCapturer(cfg.camera.id);
        cp.set_exposure_time(cfg.camera.exposure_time);
        cp.set_gain(cfg.camera.gain);
        cp.set_white_balance(cfg.camera.white_balance_red, cfg.camera.white_balance_green, cfg.camera.white_balance_blue);

        while (true)
        {
            threading::RoslikeTopic<cv::Mat>::set("capture_image", cp.next());
        }
    }

    void thread_capture(const Config &cfg)
    {
        if (cfg.capture.target == "image")
        {
            capture_image(cfg);
        }
        else if (cfg.capture.target == "video")
        {
            capture_video(cfg);
        }
        else if (cfg.capture.target == "camera")
        {
            capture_camera(cfg);
        }
        else
        {
            __LOG_ERROR_AND_EXIT("未指定捕获目标！");
        }
    }
}