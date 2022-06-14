#include "armor_icon_classifier.hpp"

#include "common/logging.hpp"

namespace rm_detect
{
ArmorIconClassifier::ArmorIconClassifier(const std::string& onnx_path)
{
  net_ = cv::dnn::readNetFromONNX(onnx_path);
}

ClassificationResult ArmorIconClassifier::classify(const cv::Mat& src)
{
  // 参考 陈君 的代码
  // rm_auto_aim\armor_detector\src\number_classifier.cpp

  // 归一化
  cv::Mat img = src.clone();
  img = img / 255.0;

  // 推理
  cv::Mat blob;
  cv::dnn::blobFromImage(img, blob, 1., cv::Size(28, 20));
  net_.setInput(blob);
  cv::Mat outputs = net_.forward();

  // softmax
  float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::Mat softmax_prob;
  cv::exp(outputs - max_prob, softmax_prob);
  float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
  softmax_prob /= sum;

  double confidence;
  cv::Point class_id_point;
  cv::minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);

  return ClassificationResult{.class_id = class_id_point.x, .confidence = confidence};
}

}  // namespace rm_detect