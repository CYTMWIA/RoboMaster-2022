#ifndef __RM_DETECT_HAMMING_MATCH_HPP__
#define __RM_DETECT_HAMMING_MATCH_HPP__

#include <opencv2/opencv.hpp>

namespace rm_detect
{
class HammingMatch
{
 private:
  cv::Mat template_;

 public:
  HammingMatch() = default;
  HammingMatch(const cv::Mat &_template);

  void set_template(const cv::Mat &_template);

  double match(const cv::Mat &img) const;
  double operator()(const cv::Mat &img) const;
};

HammingMatch make_icon_hammimg_match(const cv::Mat &icon_template);

HammingMatch make_icon_hammimg_match(std::string icon_path);
}  // namespace rm_detect

#endif