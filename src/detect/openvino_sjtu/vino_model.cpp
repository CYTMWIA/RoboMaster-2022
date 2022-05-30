#include <iostream>

#include "logging.hpp"

#include "vino_model.hpp"

namespace rmcv::detect
{
    VinoModel::VinoModel(std::string onnx_file)
    {
        net_ = cv::dnn::readNetFromONNX(onnx_file);

 	    try {
	        // 尝试使用openvino模式运行fp32模型
	        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
	        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	        cv::Mat input(640, 640, CV_8UC3);
	        auto x = cv::dnn::blobFromImage(input) / 255.;
	        net_.setInput(x);
	        net_.forward();
	        // mode = "openvino-fp32-cpu"; // 设置当前模型模式
	    } catch (cv::Exception &) {
	        // 无法使用openvino运行fp32模型，则使用默认的opencv-dnn模式。
	        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
	        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	        // mode = "dnn-fp32-cpu";      // 设置当前模型模式
	    }
    }

    VinoModel::VinoModel(std::string xml_file, std::string bin_file)
    {
        net_ = cv::dnn::readNetFromModelOptimizer(xml_file, bin_file);
        cv::Mat input(640, 640, CV_8UC3);       // 构造输入数据
        auto x = cv::dnn::blobFromImage(input);
        net_.setInput(x);
        net_.forward();
    }

    std::vector<BoundingBox> VinoModel::operator()(const cv::Mat &src) const
    {
        float scale = 640.f / std::max(src.cols, src.rows);
        cv::resize(src, src, {(int)round(src.cols * scale), (int)round(src.rows * scale)});
        cv::Mat input(640, 384, CV_8UC3, 127);
        src.copyTo(input({0, 0, src.cols, src.rows}));

        return std::vector<BoundingBox>{};
    }
}