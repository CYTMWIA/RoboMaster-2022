#ifndef __RM_DETECT_MATCHS_CLASSIFIER_HPP__
#define __RM_DETECT_MATCHS_CLASSIFIER_HPP__

#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "hamming_match.hpp"

namespace rm_detect
{
class MatchsClassifier
{
 private:
  std::unordered_map<int, HammingMatch> matchs;

 public:
  void add_match(int32_t id, HammingMatch match);
  int32_t operator()(const cv::Mat &src, double thresh = 0.66);
};
}  // namespace rm_detect

#endif