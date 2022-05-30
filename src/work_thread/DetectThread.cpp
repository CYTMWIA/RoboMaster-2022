#include "DetectThread.hpp"

#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "detect/detect.hpp"
#include "util/util.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>

namespace rmcv::work_thread
{
    DetectThread::DetectThread(const rmcv::config::Config &cfg) : 
    // detector(cfg.model.path)
    detector()
    {
    }

    void DetectThread::run()
    {
        using namespace rmcv::threading;
        using namespace rmcv::detect;

        auto fps = rmcv::util::FpsCounter();
        while (true)
        {
            // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});

            auto img = RoslikeTopic<cv::Mat>::get("capture_image");
            auto res = detector(img);
            RoslikeTopic<decltype(res)>::set("detect_result", std::move(res));
        }
    }
}