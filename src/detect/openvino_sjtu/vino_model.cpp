#include <iostream>

#include "logging.hpp"

#include "vino_model.hpp"

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

    std::vector<BoundingBox> VinoModel::operator()(const cv::Mat &img)
    {
        float scale = 640.f / std::max(img.cols, img.rows);
        cv::resize(img, img, {(int)round(img.cols * scale), (int)round(img.rows * scale)});
        cv::Mat input(640, 640, CV_8UC3, 127);
        img.copyTo(input({0, 0, img.cols, img.rows}));

        cv::Mat x = cv::dnn::blobFromImage(input);

        net_.setInput(x);
        auto y = net_.forward();

        // 模型后处理
        std::vector<BoundingBox> before_nms;
        for (int i = 0; i < y.size[1]; i++) {
            float *result = (float *) y.data + i * y.size[2];
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
}