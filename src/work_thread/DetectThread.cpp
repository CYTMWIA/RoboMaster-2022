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
#if USE_TENSORRT_SJTU
                                                                  model_(cfg.model.onnx_file)
#elif USE_OPENVINO
                                                                  model_(cfg.model.xml_file, cfg.model.bin_file)
#else
#pragma message "未指定 MODEL_RUNNER"
#endif
    {
    }

    void DetectThread::run()
    {
        using namespace rmcv::threading;
        auto fps = rmcv::util::FpsCounter();
        while (true)
        {
            RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});

            auto res = model_(RoslikeTopic<cv::Mat>::get("capture_image"));
            RoslikeTopic<decltype(res)>::set("detect_result", res);
        }
    }
}