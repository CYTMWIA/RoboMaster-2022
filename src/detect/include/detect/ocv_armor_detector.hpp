#ifndef __DETECT_OCV_ARMOR_DETECTOR_HPP__
#define __DETECT_OCV_ARMOR_DETECTOR_HPP__

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "collections.hpp"

namespace rm_detect
{
struct OcvArmorDetectorSettings
{
  std::string icon_model_path;
};

class OcvArmorDetector
{
 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
  OcvArmorDetectorSettings settings_;

 public:
  OcvArmorDetector(const OcvArmorDetectorSettings& settings);
  ~OcvArmorDetector();

  std::vector<Armor> operator()(const cv::Mat& src);
};
}  // namespace rm_detect

#endif