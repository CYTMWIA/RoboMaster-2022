#include "CaptureThread.hpp"

#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "capture/capture.hpp"
#include "util/util.hpp"

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
            target_func_ = std::bind(&CaptureThread::video, this, cfg.video.path);
        }
        else if (target_ == "camera")
        {
            target_func_ = std::bind(&CaptureThread::camera, this, cfg.camera.manufacturer, cfg.camera.exposure_time, cfg.camera.gain, cfg.camera.white_balance_red, cfg.camera.white_balance_green, cfg.camera.white_balance_blue);
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

    void CaptureThread::video(const std::string path)
    {
        using namespace rmcv::threading;

        auto cp = rmcv::capture::VideoCapture(path);

        while (true)
        {
            threading::RoslikeTopic<cv::Mat>::set("capture_image", cp.next());
        }
    }

    void CaptureThread::camera(std::string manufacturer, double exposure_time, double gain, double white_balance_red, double white_balance_green, double white_balance_blue)
    {
        using namespace rmcv::threading;
        using namespace rmcv::capture;

        std::unique_ptr<BaseCameraCapture> pcap;
        if (manufacturer=="dahua")
        {
            pcap = std::make_unique<DahuaCapture>();
        }
        else if (manufacturer=="daheng")
        {
            pcap = std::make_unique<DahengCapture>();
        }
        else
        {
            __LOG_ERROR_AND_EXIT("未指定相机厂商");
        }

        pcap->set_exposure_time(exposure_time);
        pcap->set_gain(gain);
        pcap->set_white_balance(white_balance_red, white_balance_green, white_balance_blue);

        auto fps = rmcv::util::FpsCounter();
        while (true)
        {
            RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});

            threading::RoslikeTopic<cv::Mat>::set("capture_image", pcap->next());
        }
    }

    void CaptureThread::run(void)
    {
        target_func_();
    }
}