#ifndef __VINO_MODEL_HPP__
#define __VINO_MODEL_HPP__

#include <string>
#include <vector>
#include <memory>

#include <opencv2/core.hpp>

#include "../BoundingBox.hpp"

namespace rmcv::detect
{
    class VinoModel
    {
    private:
        class Impl;
        std::unique_ptr<Impl> pimpl_;

    public:
        VinoModel() = delete;
        ~VinoModel();
        /**
         * @brief VinoModel 构造函数
         *
         * @param xml_path xml 文件路径
         * @param bin_path bin 文件路径（已弃用）
         */
        VinoModel(std::string xml_path, std::string bin_path="deprecated");

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