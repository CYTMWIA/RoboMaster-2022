#include "detect/ocv_power_rune_detector.hpp"

#include <chrono>

#include "common/logging.hpp"
#include "common/threading.hpp"
#include "ocv_utils.hpp"
namespace rm_detect
{
/**
 * @brief 旋转矩形
 *
 */
struct RawRune
{
  /*
   * width始终为长边，angle始终为长边从 向正右方射线 沿逆时针转动的角度即[0, 180)
   */
  cv::Point2f pts[4];
  cv::Point2f center;
  double width, height;
  std::chrono::time_point<std::chrono::steady_clock> time;
  std::shared_ptr<std::vector<cv::Point>> raw_contour_ptr;
  std::shared_ptr<cv::RotatedRect> raw_rotated_rect_ptr;

  RawRune() = default;
  RawRune(const cv::RotatedRect& cvrrect, const std::vector<cv::Point>& raw_contour)
      : raw_contour_ptr{std::make_shared<std::vector<cv::Point>>(raw_contour)},
        raw_rotated_rect_ptr{std::make_shared<cv::RotatedRect>(cvrrect)}
  {
    time = std::chrono::steady_clock::now();

    // 确定中心
    center = cvrrect.center;

    cvrrect.points(pts);

    // 确定长宽
    double len01 = sqrt((pts[0].x - pts[1].x) * (pts[0].x - pts[1].x) +
                        (pts[0].y - pts[1].y) * (pts[0].y - pts[1].y));
    double len03 = sqrt((pts[0].x - pts[3].x) * (pts[0].x - pts[3].x) +
                        (pts[0].y - pts[3].y) * (pts[0].y - pts[3].y));
    std::swap(pts[1], pts[3]);
    if (len01 < len03)
    {
      std::swap(len01, len03);
    }
    width = len01;
    height = len03;
  }
};

class OcvPowerRuneDetector::Impl
{
 public:
  std::vector<std::vector<RawRune>> history_;
  std::chrono::time_point<std::chrono::steady_clock> last_frame_time_;

  void frame(const cv::Mat& src)
  {
    cv::Mat bin;
    cv::cvtColor(src, bin, cv::COLOR_BGR2GRAY);
    threshold_for_low_exposure(bin);
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4)));
    // 筛选轮廓
    std::vector<std::vector<cv::Point>> cons;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, cons, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    for (int i = cons.size() - 1; i >= 0; i--)
      if (!(hierarchy[i][0] == -1 || hierarchy[i][1] == -1 || hierarchy[i][2] == -1))  // 无子轮廓
        cons.erase(cons.begin() + i);

    rm_threading::RoslikeTopic<cv::Mat>::set("debug_img_1", bin);

    std::vector<RawRune> power_runes;
    std::transform(cons.begin(), cons.end(), std::back_inserter(power_runes),
                   [](std::vector<cv::Point>& con) {
                     return RawRune{cv::minAreaRect(con), con};
                   });

    power_runes.erase(std::remove_if(power_runes.begin(), power_runes.end(),
                                     [this](const RawRune& rr)
                                     {
                                       auto area = cv::contourArea(*rr.raw_contour_ptr);
                                       if (area < 128) return true;  // 面积
                                       if (area / (rr.height * rr.width) < 0.9)
                                         return true;  // 矩形度
                                       if (rr.width / rr.height < 1 || 4 < rr.width / rr.height)
                                         return true;  // 长宽比
                                       return false;   // 合格
                                     }),
                      power_runes.end());

    auto history_size = history_.size();
    std::vector<bool> history_matched(history_size, false);
    for (const auto& pr : power_runes)
    {
      bool match_flag = false;
      for (int i = 0; i < history_size; i++)
      {
        if (history_matched[i]) continue;

        std::vector<cv::Point2f> iou;
        cv::rotatedRectangleIntersection(*pr.raw_rotated_rect_ptr,
                                         *history_[i][history_[i].size() - 1].raw_rotated_rect_ptr,
                                         iou);
        if (iou.size())
        {
          history_[i].push_back(pr);
          history_matched[i] = true;
          match_flag = true;
          break;
        }
      }
      if (!match_flag)
      {
        history_.push_back({pr});
      }
    }
    auto now = std::chrono::steady_clock::now();
    for (int i = history_size - 1; i >= 0; i--)
      if (history_[i].size() == 0 ||
          now - history_[i][history_[i].size() - 1].time > std::chrono::milliseconds(200))
      {
        history_.erase(history_.begin() + i);
      }
  }

  int get_circle_center(cv::Point2f& center, const cv::Point2f& p1, const cv::Point2f& p2,
                        const cv::Point2f& p3)
  {
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    float x3 = p3.x, y3 = p3.y;

    float a = x1 - x2;
    float b = y1 - y2;
    float c = x1 - x3;
    float d = y1 - y3;
    float e = ((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2)) / 2.0;
    float f = ((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3)) / 2.0;
    float det = b * c - a * d;
    if (fabs(det) < 1e-10)
    {
      // circle_centers.clear();
      //   __LOG_DEBUG("fuck");
      return -1;
    }
    double x0 = -(d * e - b * f) / det;
    double y0 = -(a * f - c * e) / det;
    center = cv::Point2f(x0, y0);
    return 0;
  }

  std::vector<Armor> detect(const cv::Mat& src)
  {
    if (std::chrono::steady_clock::now() - last_frame_time_ > std::chrono::milliseconds(500))
      history_.clear();
    last_frame_time_ = std::chrono::steady_clock::now();
    frame(src);

    std::vector<Armor> runes;
    for (int i = 0; i < history_.size(); i++)
    {
      // __LOG_DEBUG("{} {}", i, history_[i].size());
      if (history_[i].size() < 20) continue;

      std::vector<cv::Point2f> circle_centers;
      int step = std::max((int)history_[i].size() / 12, 1);
      for (int j = 0; j < history_[i].size() - 1; j += step)
      {
        if (history_[i][history_[i].size() - 1 - j].time - history_[i][j].time <
            std::chrono::milliseconds(100))
          break;

        cv::Point2f center;
        if (0 == get_circle_center(center, history_[i][j].center,
                                   history_[i][history_[i].size() - 1 - j].center,
                                   history_[i][history_[i].size() / 2].center))
          circle_centers.push_back(std::move(center));
      }

      if (circle_centers.size())
      {
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(circle_centers, center, radius);
        __LOG_DEBUG("{}", radius);
        if (radius < 64)
        {
          Armor res = {.color_id = 3, .tag_id = 0, .type = rm_data::ARMOR_BIG};
          res.pts[0] = history_[i][history_[i].size() - 1].pts[0];
          res.pts[1] = history_[i][history_[i].size() - 1].pts[1];
          res.pts[2] = history_[i][history_[i].size() - 1].pts[2];
          res.pts[3] = history_[i][history_[i].size() - 1].pts[3];
          runes.push_back(std::move(res));
        }
      }
    }
    return runes;
  }
};

OcvPowerRuneDetector::OcvPowerRuneDetector() : pimpl_{new Impl{}} {};
OcvPowerRuneDetector::~OcvPowerRuneDetector() = default;

std::vector<Armor> OcvPowerRuneDetector::operator()(const cv::Mat& src)
{
  return pimpl_->detect(src);
}

}  // namespace rm_detect
