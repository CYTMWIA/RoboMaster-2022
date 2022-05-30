#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "boundingbox.hpp"

#if USE_TENSORRT_SJTU
#include "tensorrt_sjtu/TRTModule.hpp"
#elif USE_OPENVINO_SJTU
#include "openvino_sjtu/vino_model.hpp"
#endif

namespace rmcv::detect
{
    class Model
    {
    private:
#if USE_TENSORRT_SJTU
        TRTModule model_;
#elif USE_OPENVINO_SJTU
        VinoModel model_;
#else
#pragma message "未指定 MODEL_RUNNER"
#endif
    public:
        Model(std::string path):model_{path} {};
        Model(std::string path_1, std::string path_2):model_{path_1, path_2} {};
        
        std::vector<BoundingBox> operator()(cv::Mat img)
        {
            return model_(img);
        }
    };
}


#endif