#ifndef __DETECT_NNARMORDETECTOR_HPP__
#define __DETECT_NNARMORDETECTOR_HPP__

#include "BoundingBox.hpp"

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

#if MODEL_RUNNER_TENSORRT_SJTU
#include "tensorrt_sjtu/TRTModule.hpp"
#elif MODEL_RUNNER_OPENVINO
#include "openvino/VinoModel.hpp"
#endif

namespace rmcv::detect
{
    class NnArmorDetector
    {
    private:
#if MODEL_RUNNER_TENSORRT_SJTU
        TRTModule model_;
#elif MODEL_RUNNER_OPENVINO
        VinoModel model_;
#elif MODEL_RUNNER_NONE
        class NoneModel
        {
        public:
            NoneModel(std::string path){};

            template <typename T>
            std::vector<BoundingBox> operator()(T img) { return std::vector<BoundingBox>(); };
        } model_;
#else
#pragma message "未指定 MODEL_RUNNER"
#endif

        cv::Mat last_frame_;
        std::vector<BoundingBox> last_detect_result_;

    public:
        NnArmorDetector(const std::string &model_path);

        std::vector<BoundingBox> operator()(const cv::Mat &frame);
    };
}

#endif