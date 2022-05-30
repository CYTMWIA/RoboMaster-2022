#ifndef __DETECT_NNARMORDETECTOR_HPP__
#define __DETECT_NNARMORDETECTOR_HPP__

#include "BoundingBox.hpp"

#include "LkOpticalFlowTracker.hpp"

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

#if ARMOR_MODEL_TENSORRT_SJTU
#include "tensorrt_sjtu/TRTModule.hpp"
#elif ARMOR_MODEL_OPENVINO
#include "openvino/VinoModel.hpp"
#endif

namespace rmcv::detect
{
    class NnArmorDetector
    {
    private:
#if ARMOR_MODEL_TENSORRT_SJTU
        TRTModule model_;
#elif ARMOR_MODEL_OPENVINO
        VinoModel model_;
#elif ARMOR_MODEL_NONE
        class NoneModel
        {
        public:
            NoneModel(std::string path){};

            template <typename T>
            std::vector<BoundingBox> operator()(T img) { return std::vector<BoundingBox>(); };
        } model_;
#else
#pragma message "未指定 ARMOR_MODEL"
#endif

        LkOpticalFlowTracker tracker_;

    public:
        NnArmorDetector(const std::string &model_path);

        std::vector<BoundingBox> operator()(const cv::Mat &frame);
    };
}

#endif