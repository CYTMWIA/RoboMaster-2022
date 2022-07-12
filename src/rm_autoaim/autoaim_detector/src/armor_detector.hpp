#ifndef __ARMOR_DETECTOR_HPP__
#define __ARMOR_DETECTOR_HPP__

#include <cv_bridge/cv_bridge.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "armor_icon_classifier.hpp"

namespace rm_autoaim
{
class ArmorDetector
{
 private:
  std::unique_ptr<ArmorIconClassifier> armor_icon_classifier_ptr_;

 public:
  void init_armor_icon_classifier(const std::string& onnx_path)
  {
    armor_icon_classifier_ptr_ = std::make_unique<ArmorIconClassifier>(onnx_path);
  }

  cv::Mat to_binary_image(const cv::Mat& src, int threshold);

  struct Lightbar
  {
    /*
     * length始终为长边，rad始终为长边从 向正右方射线 沿逆时针转动的角度即[0, 180)
     */
    cv::Point2f top, bottom, center, pts[4];
    double length, width, rad;

    Lightbar(const cv::RotatedRect& cvrrect);
  };
  struct LightbarParams
  {
    double min_contour_area;
    double max_contour_area;
    double min_rect_angle;
    double max_rect_angle;
    double min_rect_ratio;
    double max_rect_ratio;
  };
  std::vector<Lightbar> find_lightbars(const cv::Mat& bin, const LightbarParams& params);

  struct LightbarPair
  {
    std::shared_ptr<Lightbar> left_ptr;
    std::shared_ptr<Lightbar> right_ptr;
    int8_t type;  // 0: small, 1: big
    int8_t number;
    int8_t color;
  };
  struct LightbarPairParams
  {
    double max_angle_diff;
    double min_length_ratio;
    double min_center_distance_small_armor;
    double mid_center_distance;
    double max_center_distance_big_armor;
  };
  std::vector<LightbarPair> match_lightbars(const std::vector<Lightbar>& lightbars,
                                            const LightbarPairParams& params);

  cv::Mat extract_icon(const cv::Mat& src, const LightbarPair& pair);
  void filter_by_icon(const cv::Mat& src, std::vector<LightbarPair>& pairs, double threshold);

  void filter_by_color(const cv::Mat& src, std::vector<LightbarPair>& pairs);

  void draw_result_image(cv::Mat& src, const std::vector<LightbarPair>& pairs);
};
}  // namespace rm_autoaim

#endif