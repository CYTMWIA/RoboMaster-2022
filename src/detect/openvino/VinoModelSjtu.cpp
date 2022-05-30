#include <iostream>
#include <filesystem>

#include <fmt/core.h>
#include <ie_core.hpp>

#include "logging/logging.hpp"

#include "VinoModel.hpp"
#include "vino_util.hpp"

namespace rmcv::detect
{
    template<class F, class T, class ...Ts>
    T reduce(F &&func, T x, Ts ...xs) {
        if constexpr (sizeof...(Ts) > 0) {
            return func(x, reduce(std::forward<F>(func), xs...));
        } else {
            return x;
        }
    }
    
    template<class T, class ...Ts>
    T reduce_min(T x, Ts ...xs) {
        return reduce([](auto a, auto b) { return std::min(a, b); }, x, xs...);
    }
    
    template<class T, class ...Ts>
    T reduce_max(T x, Ts ...xs) {
        return reduce([](auto a, auto b) { return std::max(a, b); }, x, xs...);
    }
    
    // 判断目标外接矩形是否相交，用于nms。
    // 等效于thres=0的nms。
    static inline bool is_overlap(const cv::Point2f pts1[4], const cv::Point2f pts2[4]) {
        cv::Rect2f box1, box2;
        box1.x = reduce_min(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x);
        box1.y = reduce_min(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y);
        box1.width = reduce_max(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x) - box1.x;
        box1.height = reduce_max(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y) - box1.y;
        box2.x = reduce_min(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x);
        box2.y = reduce_min(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y);
        box2.width = reduce_max(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x) - box2.x;
        box2.height = reduce_max(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y) - box2.y;
        return (box1 & box2).area() > 0;
    }
    
    static inline int argmax(const float *ptr, int len) {
        int max_arg = 0;
        for (int i = 1; i < len; i++) {
            if (ptr[i] > ptr[max_arg]) max_arg = i;
        }
        return max_arg;
    }
    
    float inv_sigmoid(float x) {
        return -std::log(1 / x - 1);
    }
    
    float sigmoid(float x) {
        return 1 / (1 + std::exp(-x));
    }

    class VinoModel::Impl
    {
        private:
            InferenceEngine::Core core_;
            InferenceEngine::InputsDataMap input_info_;
            InferenceEngine::OutputsDataMap output_info_;
            InferenceEngine::ExecutableNetwork network_;
            InferenceEngine::InferRequest infer_request_;

        public:
            Impl(std::string xml_path, std::string bin_path)
            {
                auto cache_dir = std::filesystem::path(xml_path).parent_path()/"openvino_cache";
                core_.SetConfig({{CONFIG_KEY(CACHE_DIR), cache_dir}});

                std::vector<std::string> devices = core_.GetAvailableDevices();
                __LOG_INFO("Available Devices: {}", fmt::join(devices, ", "));

                auto net_info = core_.ReadNetwork(xml_path);
                // 输入设置
                input_info_ = net_info.getInputsInfo();
                auto input = input_info_.begin()->second;
                input->setPrecision(InferenceEngine::Precision::I8);
                // 输出设置
                output_info_ = net_info.getOutputsInfo();
                
                network_ = core_.LoadNetwork(xml_path, "GPU");

                infer_request_ = network_.CreateInferRequest();

                cv::Mat test(640, 640, CV_8UC3);
                operator()(test);
            }

            std::vector<BoundingBox> operator()(const cv::Mat &img)
            {
                float scale = 640.f / std::max(img.cols, img.rows);
                cv::resize(img, img, {(int)round(img.cols * scale), (int)round(img.rows * scale)});
                cv::Mat input_img(640, 640, CV_8UC3, 127);
                img.copyTo(input_img({0, 0, img.cols, img.rows}));

                auto input_blob = infer_request_.GetBlob(input_info_.begin()->first);
                image2blob(input_img, input_blob);

                infer_request_.Infer();

                auto output = infer_request_.GetBlob(output_info_.begin()->first);
                const float *y = output->cbuffer().as<const float *>();
                
                // 模型后处理
                std::vector<BoundingBox> before_nms;
                for (int i = 0; i < 25200; i++) {
                    float *result = (float *) y + i * 22;
                    if (result[8] < inv_sigmoid(0.5)) continue;
                    BoundingBox box;
                    for (int i = 0; i < 4; i++) {
                        box.pts[i].x = (result[i * 2 + 0]) / scale;
                        box.pts[i].y = (result[i * 2 + 1]) / scale;
                    }
                    box.color_id = argmax(result + 9, 4);
                    box.tag_id = argmax(result + 13, 9);
                    box.confidence = sigmoid(result[8]);
                    before_nms.push_back(box);
                }
                std::sort(before_nms.begin(), before_nms.end(), [](BoundingBox &b1, BoundingBox &b2) {
                    return b1.confidence > b2.confidence;
                });
                std::vector<BoundingBox> boxes;
                boxes.reserve(before_nms.size());
                std::vector<bool> is_removed(before_nms.size());
                for (int i = 0; i < before_nms.size(); i++) {
                    if (is_removed[i]) continue;
                    boxes.push_back(before_nms[i]);
                    for (int j = i + 1; j < before_nms.size(); j++) {
                        if (is_removed[j]) continue;
                        if (is_overlap(before_nms[i].pts, before_nms[j].pts)) is_removed[j] = true;
                    }
                }

                return boxes;
            }
    };

    VinoModel::VinoModel(std::string xml_path, std::string bin_path):  pimpl_(std::make_unique<Impl>(xml_path, bin_path))
    {
    }
    VinoModel::~VinoModel() = default;

    std::vector<BoundingBox> VinoModel::operator()(const cv::Mat &img)
    {
        return pimpl_->operator()(img);
    }
}