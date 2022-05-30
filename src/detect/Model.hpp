#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "BoundingBox.hpp"

#if MODEL_RUNNER_TENSORRT_SJTU
#include "tensorrt_sjtu/TRTModule.hpp"
#elif MODEL_RUNNER_OPENVINO
#include "openvino/VinoModel.hpp"
#endif

namespace rmcv::detect
{
    class NoneModel
    {
    public:
        NoneModel(std::string path){};

        template <typename T>
        std::vector<BoundingBox> operator()(T img) { return std::vector<BoundingBox>(); };
    };

    class Model
    {
    private:
#if MODEL_RUNNER_TENSORRT_SJTU
        TRTModule model_;
#elif MODEL_RUNNER_OPENVINO
        VinoModel model_;
#elif MODEL_RUNNER_NONE
        NoneModel model_;
#else
#pragma message "未指定 MODEL_RUNNER"
#endif

    public:
        Model(std::string path) : model_{path} {};
        std::vector<BoundingBox> operator()(cv::Mat img);
    };
}

#endif