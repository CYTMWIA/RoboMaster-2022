#include "power_rune_detector.hpp"

#include <algorithm>
#include <memory>
#include <numeric>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <tuple>
#include <type_traits>
#include <vector>

#include "../geometry/utils.hpp"
#include "utils.hpp"

namespace rm_autoaim
{
PowerRuneDetector::PowerRuneDetector(const PowerRuneDetectorConfig& config) : config_(config) {}

std::vector<PowerRuneDetector::Logo> PowerRuneDetector::find_logos(
    const std::vector<std::vector<cv::Point>>& contours)
{
  std::vector<Logo> center_logos;
  for (const auto& con : contours)
  {
    double area = cv::contourArea(con);
    if (area < 100) continue;
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(con, center, radius);
    center_logos.emplace_back(center, radius);
  }

  filter_emplace(center_logos,
                 [&](const auto& testing)
                 {
                   if (testing.radius < 5 || 30 < testing.radius) return false;

                   for (int j = 0; j < (int)center_logos.size(); j++)
                   {
                     const auto& another = center_logos[j];
                     if (&another == &testing) continue;

                     float out_distance = cv::norm(testing.center - another.center) -
                                          (testing.radius + another.radius);
                     if (out_distance < 5)
                     {
                       return false;
                     }
                   }
                   return true;
                 });
  return center_logos;
}

std::vector<PowerRuneDetector::ArmorRect> PowerRuneDetector::find_armors(
    const std::vector<std::vector<cv::Point>>& contours, const std::vector<cv::Vec4i>& hierarchy)
{
  std::vector<PowerRuneDetector::ArmorRect> armors;
  for (int i = contours.size() - 1; i >= 0; i--)
    if (hierarchy[i][0] == -1 || hierarchy[i][1] == -1 || hierarchy[i][2] == -1)  // 无子轮廓
    {
      auto rr = cv::minAreaRect(contours[i]);
      cv::Point2f pts[4];
      rr.points(pts);
      float short_line = cv::norm(pts[0] - pts[1]), long_line = cv::norm(pts[0] - pts[3]);
      if (long_line < short_line)
      {
        std::swap(short_line, long_line);
        std::swap(pts[1], pts[3]);
      }
      if (long_line < 30 || 80 < long_line) continue;
      float ratio = long_line / short_line;
      if (ratio < 1.7 || 3.2 < ratio) continue;

      armors.emplace_back(rr.center, pts[0], pts[1], pts[2], pts[3]);
    }
  return armors;
}

std::vector<PowerRune> PowerRuneDetector::match_logos_and_armors(
    const cv::Mat& bin, const std::vector<Logo>& logos, const std::vector<ArmorRect>& armors)
{
  std::vector<PowerRune> power_runes;
  for (const auto& logo : logos)
  {
    for (int i = 0; i < (int)armors.size(); i++)
    {
      const auto& armor = armors[i];

      auto v1 = logo.center - armor.center;
      auto v2 = armor.pts[1] - armor.pts[2];
      auto angle = to_angle(acos(v1.dot(v2) / abs(cv::norm(v1) * cv::norm(v2))));
      if (angle < 75 || 105 < angle) continue;

      // 流水灯判断
      auto it = cv::LineIterator(bin, logo.center, armor.center);
      int px_count = it.count;  // 线上的像素总数
      std::vector<int> seq;
      std::remove_reference<decltype(**it)>::type last_px;
      for (int j = 0; j < it.count; j++, ++it)
      {
        auto px = (**it);
        if (seq.empty() || last_px != px)
          seq.emplace_back(1);
        else
          seq[seq.size() - 1]++;
        last_px = px;
      }
      // seq中有x个以上的、连续的、与y相差不超过z的项，即流水灯
      int idx = 0, max_count = 0, max_px_count = 0;
      while (idx < (int)seq.size())
      {
        int count = 0;
        while (idx + count < (int)seq.size() && std::abs(seq[idx + count] - 7) <= 6) count++;
        if (count > max_count)
        {
          max_count = count;
          max_px_count = std::accumulate(seq.begin() + idx, seq.begin() + idx + count, 0);
        }
        idx += count + 1;
      }
      max_count += 1;  // 上边的是没有计算起始项的，在这加回来
      if (max_count >= 5 && max_px_count >= px_count * 0.5)
      {
        std::vector<PowerRune> prs;
        // 可激活扇叶
        PowerRune apr = {.armor_pts{armor.pts[0], armor.pts[1], armor.pts[2], armor.pts[3]},
                         .armor_center = armor.center,
                         .logo = logo.center,
                         .activating = true};
        prs.push_back(apr);
        auto alength = distance(apr.armor_center, apr.logo);
        // 其他扇叶
        for (int j = 0; j < (int)armors.size(); j++)
        {
          if (i == j) continue;
          const auto& armor = armors[j];

          auto len = distance(armor.center, logo.center);
          auto len_ratio = len / alength;
          if (len_ratio < 0.9 || 1.1 < len_ratio) continue;

          auto v1 = logo.center - armor.center;
          auto v2 = armor.pts[1] - armor.pts[2];
          auto angle = to_angle(acos(v1.dot(v2) / abs(cv::norm(v1) * cv::norm(v2))));
          if (angle < 75 || 105 < angle) continue;

          PowerRune pr = {.armor_pts{armor.pts[0], armor.pts[1], armor.pts[2], armor.pts[3]},
                          .armor_center = armor.center,
                          .logo = logo.center,
                          .activating = false};
          prs.push_back(pr);
        }
        if (prs.size() > power_runes.size() && prs.size() < 6) power_runes = std::move(prs);
      }
    }
  }
  return power_runes;
}

std::vector<PowerRune> PowerRuneDetector::detect(const cv::Mat& src)
{
  cv::Mat bin = to_binary(src, config_.binary_thresh);

  // 查找轮廓
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
  // 中心R标志
  auto logos = find_logos(contours);
  // 装甲板
  auto armors = find_armors(contours, hierarchy);
  // 两两匹配
  auto res = match_logos_and_armors(bin, logos, armors);

  cv::Mat show(src);
  for (const auto& logo : logos)
    cv::circle(show, logo.center, logo.radius, cv::Scalar(255, 0, 0), 4);
  for (const auto& armor : armors)
  {
    for (int i = 0; i < 4; i++)
      cv::line(show, armor.pts[i], armor.pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 4);
  }
  for (const auto& r : res)
  {
    cv::line(show, r.armor_center, r.logo, cv::Scalar(100, 255, 0), 4);
  }
  nerv::Topic<cv::Mat>::set("debug_img_1", show);

  return res;
}
}  // namespace rm_autoaim