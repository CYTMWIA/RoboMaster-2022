#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "BoundingBox.hpp"

#if USE_TENSORRT_SJTU
#include "tensorrt_sjtu/TRTModule.hpp"
#elif USE_OPENVINO_SJTU
#include "openvino_sjtu/VinoModel.hpp"
#endif

namespace rmcv::detect
{
    class Model
    {
#if USE_TENSORRT_SJTU
    private:
        TRTModule model_;

    public:
        Model(std::string onnx_path) : model_{onnx_path} {};
#elif USE_OPENVINO_SJTU
    private:
        VinoModel model_;

    public:
        Model(std::string xml_path, std::string bin_path) : model_{xml_path, bin_path} {};
#else
#pragma message "未指定 MODEL_RUNNER"
#endif
    public:
        std::vector<BoundingBox> operator()(cv::Mat img);
    };
}

#endif