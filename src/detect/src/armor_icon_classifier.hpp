#ifndef __ARMOR_ICON_CLASSIFIER_HPP__
#define __ARMOR_ICON_CLASSIFIER_HPP__

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "detect/collections.hpp"

namespace rm_detect
{
struct ClassificationResult
{
  int class_id;
  double confidence;
};

class ArmorIconClassifier
{
 private:
  cv::dnn::Net net_;

 public:
  ArmorIconClassifier(const std::string& onnx_path);
  ClassificationResult classify(const cv::Mat& src);
};
}  // namespace rm_detect

#endif