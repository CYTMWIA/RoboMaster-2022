#include "DetectThread.hpp"

#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "detect/detect.hpp"

#include <opencv2/opencv.hpp>

namespace rmcv::work_thread
{
    DetectThread::DetectThread(const rmcv::config::Config &cfg)
    {
        using namespace rmcv::detect;

        if (cfg.model.onnx_file != "")
        {
            pmodel = std::make_unique<Model>(cfg.model.onnx_file);
        }
        else if (cfg.model.xml_file != "" && cfg.model.bin_file != "")
        {
            pmodel = std::make_unique<Model>(cfg.model.xml_file, cfg.model.bin_file);
        }
        else
        {
            __LOG_ERROR_AND_EXIT("未指定模型文件");
        }
    }

    void DetectThread::run()
    {
        using namespace rmcv::threading;

        while (true)
        {
            auto res = pmodel->operator()(RoslikeTopic<cv::Mat>::get("capture_image"));
            RoslikeTopic<decltype(res)>::set("detect_result", std::move(res));
        }
    }

    void DetectThread::up()
    {
        auto running = std::bind(&DetectThread::run, this);
        thread_ = std::thread(running);
    }
}