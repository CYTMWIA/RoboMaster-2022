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

    void CaptureThread::init_camera(std::unique_ptr<rmcv::capture::BaseCameraCapture> &pcap)
    {
        using namespace rmcv::capture;

        if (camera_.manufacturer=="dahua")
        {
            pcap = std::make_unique<DahuaCapture>();
        }
        else if (camera_.manufacturer=="daheng")
        {
            pcap = std::make_unique<DahengCapture>();
        }
        else
        {
            __LOG_ERROR_AND_EXIT("未指定相机厂商");
        }

        pcap->set_exposure_time(camera_.exposure_time);
        pcap->set_gain(camera_.gain);
        pcap->set_white_balance(camera_.white_balance_red, camera_.white_balance_green, camera_.white_balance_blue);
    }

    void CaptureThread::camera()
    {
        using namespace rmcv::threading;
        using namespace rmcv::capture;

        std::unique_ptr<BaseCameraCapture> pcap;
        init_camera(pcap);

        auto fps = rmcv::util::FpsCounter();
        while (true)
        {
            // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});
            try
            {
                threading::RoslikeTopic<cv::Mat>::set("capture_image", pcap->next());
            }
            catch (std::runtime_error e)
            {
                __LOG_ERROR("{}", e.what());
                init_camera(pcap);
            }
        }
    }

    void CaptureThread::run(void)
    {
        target_func_();
    }
}