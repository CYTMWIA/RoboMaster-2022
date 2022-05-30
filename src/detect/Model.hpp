#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "BoundingBox.hpp"

#if USE_TENSORRT_SJTU
#include "tensorrt_sjtu/TRTModule.hpp"
#elif USE_OPENVINO
#include "openvino/VinoModel.hpp"
#endif

namespace rmcv::detect
{
    class Model
    {
    private:
#if USE_TENSORRT_SJTU
        TRTModule model_;
#elif USE_OPENVINO
        VinoModel model_;
#else
#pragma message "未指定 MODEL_RUNNER"
#endif

    public:
        Model(std::string path) : model_{path} {};
        std::vector<BoundingBox> operator()(cv::Mat img);
    };
}

#endif