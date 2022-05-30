#ifndef __VINO_MODEL_HPP__
#define __VINO_MODEL_HPP__

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "../boundingbox.hpp"

namespace rmcv::detect
{
     class VinoModel
     {
     private:
        cv::dnn::Net net_;
     public:
        VinoModel() = delete;

        /**
         * @brief VinoModel 构造函数
         * 
         * @param onnx_file 模型路径
         */
        VinoModel(std::string onnx_file);

        /**
         * @brief VinoModel 构造函数
         * 
         * @param xml_file xml 文件路径
         * @param bin_file bin 文件路径
         */
        VinoModel(std::string xml_file, std::string bin_file);

        /**
         * @brief 推理
         * 
         * @param img 图像
         * @return std::vector<BoundingBox> 检测结果
         */
        std::vector<BoundingBox> operator()(const cv::Mat &img);
     };
}

#endif