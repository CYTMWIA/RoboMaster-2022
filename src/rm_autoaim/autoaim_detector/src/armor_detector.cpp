#include "armor_detector.hpp"

#include <cstddef>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "utils.hpp"

namespace rm_autoaim
{
ArmorDetector::Lightbar::Lightbar(const cv::RotatedRect& cvrrect)
{
  // 确定 中心
  center = cvrrect.center;
  // 矩形四顶点
  cvrrect.points(pts);

  // 确定 长宽
  double len01 = sqrt((pts[0].x - pts[1].x) * (pts[0].x - pts[1].x) +
                      (pts[0].y - pts[1].y) * (pts[0].y - pts[1].y));
  double len03 = sqrt((pts[0].x - pts[3].x) * (pts[0].x - pts[3].x) +
                      (pts[0].y - pts[3].y) * (pts[0].y - pts[3].y));
  if (len01 < len03)
  {
    std::swap(pts[1], pts[3]);
    std::swap(len01, len03);
  }
  length = len01;
  width = len03;

  // 确定 倾斜弧度
  if (pts[0].y > pts[1].y) std::swap(pts[0], pts[1]);
  rad = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);

  // 确定 上下顶点
  float dx = length / 2 * cos(rad), dy = length / 2 * sin(rad);
  top = cv::Point2f(center.x - dx, center.y - dy);
  bottom = cv::Point2f(center.x + dx, center.y + dy);
}

cv::Mat ArmorDetector::to_binary_image(const cv::Mat& src, int threshold)
{
  cv::Mat bin;
  cv::cvtColor(src, bin, cv::COLOR_BGR2GRAY);
  if (threshold)
  {
    cv::threshold(bin, bin, threshold, 255, cv::THRESH_BINARY);
    return bin;
  }

  // 自动阈值（慢！）
  // 灰度曲线
  int gray_count[256];
  for (int i = 0; i < 256; i++) gray_count[i] = 0;
  for (int i = 0; i < bin.rows; i++)
    for (int j = 0; j < bin.cols; j++) gray_count[*(bin.data + i * bin.cols + j)] += 1;

  double gray_curve[256];
  for (int i = 0; i < 256; i++) gray_curve[i] = (gray_count[i] * 1.0) / (bin.cols * bin.rows);
  int thresh = 127;
  for (int i = 25; i < 250; i++)
  {
#define GRAD(x) (std::abs(gray_curve[(x) + 1] - gray_curve[(x)-1]) / 2.0)
    if (GRAD(i) < 0.05)
    {
      int j = i + 1;
      while (GRAD(j) < 0.05) j++;
      thresh = (i + j) * 0.3;
      if (j - i > 64) break;
    }
#undef GRAD
  }
  thresh = std::min(255, thresh);

  cv::threshold(bin, bin, thresh, 255, cv::THRESH_BINARY);
  return bin;
}

std::vector<ArmorDetector::Lightbar> ArmorDetector::find_lightbars(const cv::Mat& bin,
                                                                   const LightbarParams& params)
{
  std::vector<std::vector<cv::Point>> cons;
  std::vector<cv::Vec4i> hierarchy;  // unused
  cv::findContours(bin, cons, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  filter_emplace(cons,
                 [&](const std::vector<cv::Point>& con)
                 {
                   auto area = cv::contourArea(con);
                   return params.min_contour_area <= area && area <= params.max_contour_area;
                 });

  std::vector<Lightbar> lightbars;
  std::transform(cons.begin(), cons.end(), std::back_inserter(lightbars),
                 [](std::vector<cv::Point>& con) { return Lightbar{cv::minAreaRect(con)}; });
  filter_emplace(
      lightbars,
      [&](const Lightbar& rr)
      {
        if (rr.rad < to_rad(params.min_rect_angle) || to_rad(params.max_rect_angle) < rr.rad)
          return false;
        auto ratio = rr.length / rr.width;
        if (ratio < params.min_rect_ratio || params.max_rect_ratio < ratio) return false;
        return true;
      });

  std::sort(lightbars.begin(), lightbars.end(),
            [](auto& r1, auto& r2)
            { return r1.center.x < r2.center.x; });  // 按照中心x升序（从左到右）

  return lightbars;
}

std::vector<ArmorDetector::LightbarPair> ArmorDetector::match_lightbars(
    const std::vector<Lightbar>& lightbars, const LightbarPairParams& params)
{
  // 参考：
  // RoboMaster创梦之翼战队自瞄系统设计
  // https://zhuanlan.zhihu.com/p/416449365
  // 陈君的armor_detector
  // https://github.com/chenjunnn/rm_auto_aim/tree/main/armor_detector
  std::vector<LightbarPair> res;
  for (int i = 0; i < (long)lightbars.size() - 1; i++)
    for (int j = i + 1; j < (long)lightbars.size(); j++)
    {
      auto& left = lightbars[i];
      auto& right = lightbars[j];

      LightbarPair pair;
      pair.left_ptr = std::make_shared<Lightbar>(left);
      pair.right_ptr = std::make_shared<Lightbar>(right);

#define ASSERT(name, x)                                                        \
  if (!(x))                                                                    \
  {                                                                            \
    /*RCLCPP_DEBUG(rclcpp::get_logger("armor detect"), "JUMP OUT %s", name);*/ \
    continue;                                                                  \
  }

      ASSERT("两灯条角度差", abs(left.rad - right.rad) <= to_rad(params.max_angle_diff));

      ASSERT("两灯条长度比", params.min_length_ratio <= std::min(left.length, right.length) /
                                                            std::max(left.length, right.length));

      //   auto v1 = left.top - left.bottom;
      //   auto v2 = right.bottom - left.bottom;
      //   auto ra = acos(v1.dot(v2) / abs(cv::norm(v1) * cv::norm(v2)));
      //   ASSERT("组成四边形的角", M_PI / 6.0 < ra && ra < M_PI * (5.0 / 6.0));

      // 灯条中心距与灯条均长，顺便猜测装甲板类型
      double ratio = cv::norm(left.center - right.center) / ((left.length + right.length) / 2.0);
      ASSERT("灯条中心距/灯条均长", params.min_center_distance_small_armor <= ratio &&
                                        ratio <= params.max_center_distance_big_armor)
      if (params.mid_center_distance < ratio)
        pair.type = 1;  // 大装甲板
      else
        pair.type = 0;

      res.push_back(pair);
    }
  return res;
}

cv::Mat ArmorDetector::extract_icon(const cv::Mat& src, const LightbarPair& pair)
{
  const cv::Size roi_size = cv::Size(20, 28);
  const int lightbar_length = 14;
  const int armor_small_width = 30;
  const int armor_big_width = 54;
  const int warp_lightbar_top_y = (roi_size.height - lightbar_length) / 2;
  const int warp_lightbar_bottom_y = warp_lightbar_top_y + lightbar_length;

  int armor_width = armor_small_width;
  if (pair.type == 1) armor_width = armor_big_width;
  int warp_lightbar_left_x = -(armor_width - roi_size.width) / 2;
  int warp_lightbar_right_x = warp_lightbar_left_x + armor_width;

  std::vector<cv::Point2f> icon_vertexs_src = {pair.left_ptr->top, pair.left_ptr->bottom,
                                               pair.right_ptr->bottom, pair.right_ptr->top};

  std::vector<cv::Point2f> icon_vertexs_dst = {
      cv::Point2f(warp_lightbar_left_x, warp_lightbar_top_y),
      cv::Point2f(warp_lightbar_left_x, warp_lightbar_bottom_y),
      cv::Point2f(warp_lightbar_right_x, warp_lightbar_bottom_y),
      cv::Point2f(warp_lightbar_right_x, warp_lightbar_top_y)};

  cv::Mat icon_img;
  cv::warpPerspective(src, icon_img,
                      cv::getPerspectiveTransform(icon_vertexs_src, icon_vertexs_dst), roi_size);

  cv::cvtColor(icon_img, icon_img, cv::COLOR_BGR2GRAY);
  cv::threshold(icon_img, icon_img, 0, 255, cv::THRESH_OTSU);
  return icon_img;
}

void ArmorDetector::filter_by_icon(const cv::Mat& src, std::vector<LightbarPair>& pairs,
                                   double threshold)
{
  filter_emplace(pairs,
                 [&](LightbarPair& pair)
                 {
                   cv::Mat icon = extract_icon(src, pair);
                   auto res = armor_icon_classifier_ptr_->classify(icon);
                   pair.number = res.class_id;
                   return res.confidence >= threshold;
                 });
}

void ArmorDetector::filter_by_color(const cv::Mat& src, std::vector<LightbarPair>& pairs)
{
  filter_emplace(pairs,
                 [&](LightbarPair& pair)
                 {
                   auto& left = *pair.left_ptr;
                   int blue = 0, red = 0;
                   for (int i = 0; i < 4; i++)
                   {
                     auto it = cv::LineIterator(src, left.pts[i], left.pts[(i + 1) % 4], 8);
                     for (int j = 0; j < it.count; j++, ++it)
                     {
                       auto& px = (*(const cv::Vec3b*)*it);
                       blue += px[0];
                       red += px[2];
                     }
                   }
                   pair.color = red > blue;
                   return true;  // 暂时不做实际过滤，待有明显颜色误识别问题再做处理
                 });
}

void ArmorDetector::draw_result_image(cv::Mat& src, const std::vector<LightbarPair>& pairs)
{
  const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
  const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
  const cv::Scalar RED = cv::Scalar(0, 0, 255);

  for (const auto& pair : pairs)
  {
    cv::line(src, pair.left_ptr->top, pair.right_ptr->bottom, GREEN);
    cv::line(src, pair.left_ptr->bottom, pair.right_ptr->top, GREEN);

    std::string label = std::to_string(pair.number);
    if (pair.type == 0)
      label += " S";
    else if (pair.type == 1)
      label += " B";
    auto text_color = RED;
    if (pair.color == 0) text_color = BLUE;
    cv::putText(src, label, pair.left_ptr->top, cv::FONT_HERSHEY_PLAIN, 2, text_color);
  }
}

}  // namespace rm_autoaim
