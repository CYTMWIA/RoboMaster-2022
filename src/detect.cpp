#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "detect/model.hpp"

#include "detect.hpp"

namespace rmcv
{
    using namespace config;
    using namespace threading;

    void thread_detect(const Config &cfg)
    {
        detect::Model *model;

        if (cfg.model.onnx_file != "")
        {
            model = new detect::Model{cfg.model.onnx_file};
        }
        else if (cfg.model.xml_file != "" && cfg.model.bin_file != "")
        {
            model = new detect::Model{cfg.model.xml_file, cfg.model.bin_file};
        }
        else
        {
            __LOG_ERROR_AND_EXIT("未指定模型文件");
        }

        while (true)
        {
            auto res = model->operator()(RoslikeTopic<cv::Mat>::get("capture_image"));
            RoslikeTopic<decltype(res)>::set("detect_result", std::move(res));
        }

        delete model;
    }
}