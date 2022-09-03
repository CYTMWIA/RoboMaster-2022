#ifndef __DETECTOR_POWER_RUNE_DETECTOR_HPP__
#define __DETECTOR_POWER_RUNE_DETECTOR_HPP__

#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "../interfaces.hpp"

namespace rm_autoaim
{
struct PowerRuneDetectorConfig
{
  int binary_thresh;
};

class PowerRuneDetector
{
 private:
  struct Logo
  {
    cv::Point2f center;
    float radius;
  };

  struct ArmorRect
  {
    cv::Point2f center;
    cv::Point2f pts[4];
    inline ArmorRect(const cv::Point2f& center_, const cv::Point2f& pts0, const cv::Point2f& pts1,
                     const cv::Point2f& pts2, const cv::Point2f& pts3)
        : center(center_), pts{pts0, pts1, pts2, pts3}
    {
    }
  };

  PowerRuneDetectorConfig config_;
  std::vector<Logo> find_logos(const std::vector<std::vector<cv::Point>>& contours);
  std::vector<ArmorRect> find_armors(const std::vector<std::vector<cv::Point>>& contours,
                                     const std::vector<cv::Vec4i>& hierarchy);
  std::vector<PowerRune> match_logos_and_armors(const cv::Mat& bin, const std::vector<Logo>& logos,
                                                const std::vector<ArmorRect>& armors);

 public:
  explicit PowerRuneDetector(const PowerRuneDetectorConfig& config);
  std::vector<PowerRune> detect(const cv::Mat& src);
};

}  // namespace rm_autoaim

#endif
