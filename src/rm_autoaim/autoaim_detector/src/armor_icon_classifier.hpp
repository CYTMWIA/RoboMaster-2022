#ifndef __ARMOR_ICON_CLASSIFIER_HPP__
#define __ARMOR_ICON_CLASSIFIER_HPP__

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rm_autoaim
{
struct ClassificationResult
{
  int class_id;
  double confidence;

  inline ClassificationResult(int _class_id, double _confidence)
      : class_id(_class_id), confidence(_confidence)
  {
  }
};

class ArmorIconClassifier
{
 private:
  cv::dnn::Net net_;

 public:
  ArmorIconClassifier(const std::string& onnx_path);
  ClassificationResult classify(const cv::Mat& src);
};
}  // namespace rm_autoaim

#endif