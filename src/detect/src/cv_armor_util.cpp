#include "detect/cv_armor_util.hpp"

#include <algorithm>
#include <vector>

#include "common/threading.hpp"

namespace rm_detect
{
double judge_lightbars_pair(const BoundingBox &lightbars_bbox)
{
  // 参考：RoboMaster创梦之翼战队自瞄系统设计
  // https://zhuanlan.zhihu.com/p/416449365
  using namespace std;
  double c = 0, total_score = 0;

  double tmp_c;
#define ADD_CONFIDENCE(weight, minv, x) \
  total_score += weight;                \
  tmp_c = (double)(weight) * (x);       \
  if (tmp_c < minv)                     \
    return 0;                           \
  else                                  \
    c += tmp_c;

  const cv::Point2f *pts = lightbars_bbox.pts;
  auto v1 = pts[0] - pts[1];
  auto l1 = cv::norm(v1);
  auto v2 = pts[3] - pts[2];
  auto l2 = cv::norm(v2);
  auto v3 = (pts[0] + pts[1]) / 2.0 - (pts[2] + pts[3]) / 2.0;
  auto l3 = cv::norm(v3);

  // 两灯条角度差
  ADD_CONFIDENCE(300, 150, (v1 / l1).dot(v2 / l2));

  // 两灯条连接端点后应接近矩形而非平行四边形
  ADD_CONFIDENCE(100, 30, 1 - (v1 / l1).dot(v3 / l3));

  // 两灯条间距应在一定范围内
  ADD_CONFIDENCE(100, 30, 1 - (l3 / min(l1, l2) - 1) / 5);

  // 两灯条长度比应在一定范围内
  ADD_CONFIDENCE(100, 30, 1 - (max(l1, l2) / min(l1, l2) - 1) / 2.0);

#undef ADD_CONFIDENCE
  c = max(0.0, c) / total_score;
  std::cout << "CONF: " << c << std::endl;

  return c;
}

double judge_lightbars_pair(const RRect &left, const RRect &right)
{
  return judge_lightbars_pair(make_boundingbox(left, right));
}

void match_lightbars(std::vector<RRect> &rrects, std::vector<LightbarMatchResult> &results)
{
  if (rrects.empty()) return;

  std::sort(rrects.begin(), rrects.end(), [](auto &r1, auto &r2) {
    return r1.center.x < r2.center.x;
  });  // 按照中心x升序（从左到右）

  for (int i = 0; i < rrects.size() - 1; i++)
    for (int j = i + 1; j < rrects.size(); j++)
    {
      LightbarMatchResult res = {i, j, judge_lightbars_pair(rrects[i], rrects[j])};

      results.push_back(std::move(res));
    }
  if (results.empty()) return;

  std::sort(results.begin(), results.end(),
            [](auto &p1, auto &p2) { return p2.confidence < p1.confidence; });  // 按置信度倒序
}

void fix_boundingbox(BoundingBox &bbox, const cv::Mat &img)
{
  using namespace rm_threading;
  /*
   * 使用OpenCV重新定位检测结果中的四个点
   */

  cv::Range range_row, range_col;  // 外接矩形
  range_row.start =
      std::min(bbox.pts[0].y, std::min(bbox.pts[1].y, std::min(bbox.pts[2].y, bbox.pts[3].y)));
  range_row.end =
      std::max(bbox.pts[0].y, std::max(bbox.pts[1].y, std::max(bbox.pts[2].y, bbox.pts[3].y)));
  range_col.start =
      std::min(bbox.pts[0].x, std::min(bbox.pts[1].x, std::min(bbox.pts[2].x, bbox.pts[3].x)));
  range_col.end =
      std::max(bbox.pts[0].x, std::max(bbox.pts[1].x, std::max(bbox.pts[2].x, bbox.pts[3].x)));

  // 范围扩大
  const float ws = 0.25, hs = 0.25;
  range_row.start = std::max(0, range_row.start - int((range_row.end - range_row.start) * hs / 2));
  range_row.end =
      std::min(img.rows, range_row.end + int((range_row.end - range_row.start) * hs / 2));
  range_col.start = std::max(0, range_col.start - int((range_col.end - range_col.start) * ws / 2));
  range_col.end =
      std::min(img.cols, range_col.end + int((range_col.end - range_col.start) * ws / 2));

  cv::Mat roi = img(range_row, range_col);

  // 统一宽度
  double scale = 100.0 / roi.cols;
  cv::resize(roi, roi, cv::Size(100, roi.rows * scale));

  // 自动二值化，用尽可能小的阈值达成区域内只有两个轮廓（使灯条区域最大）
  cv::Mat gray, thr;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  std::vector<std::vector<cv::Point>> contours, cons;
  int32_t lo = 100, hi = 255, mid;
  while (lo + 16 < hi)
  {
    contours.clear();

    mid = (lo + hi) / 2;
    cv::threshold(gray, thr, mid, 255, cv::THRESH_BINARY);
    cons = find_external_contours(thr);
    std::copy_if(cons.begin(), cons.end(), std::back_inserter(contours),
                 [](auto &c) { return cv::contourArea(c) > 4; });

    if (2 < contours.size())
    {
      lo = mid + 1;
    }
    else
    {
      hi = mid;
    }
  }

  // 灯条匹配
  std::vector<RRect> rrects;
  std::vector<LightbarMatchResult> pairs;
  if (contours.size() == 2)
  {
    for (const auto &con : contours)
    {
      rrects.push_back(RRect(cv::minAreaRect(con)));
    }
    // RoslikeTopic<cv::Mat>::set("debug_img_1", thr);
    // RoslikeTopic<cv::Mat>::set("debug_img_2", awakenlion_threshold(roi));
  }
  else
  {
    roi = awakenlion_threshold(roi);
    cv::morphologyEx(roi, roi, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 4)));
    contours = find_external_contours(roi);

    for (const auto &con : contours)
    {
      // 筛选轮廓
      if (cv::contourArea(con) < 16) continue;

      rrects.push_back(RRect(cv::minAreaRect(con)));
    }
    if (rrects.size() < 2) return;
  }
  match_lightbars(rrects, pairs);

  if (pairs.size() && (contours.size() == 2 || pairs[0].confidence > 0.6))
  {
    // std::cout << "修正实行" << std::endl;
    auto b = make_boundingbox(rrects[pairs[0].left_idx], rrects[pairs[0].right_idx]);
    bbox.pts[0] =
        cv::Point2f(range_col.start + b.pts[0].x / scale, range_row.start + b.pts[0].y / scale);
    bbox.pts[1] =
        cv::Point2f(range_col.start + b.pts[1].x / scale, range_row.start + b.pts[1].y / scale);
    bbox.pts[2] =
        cv::Point2f(range_col.start + b.pts[2].x / scale, range_row.start + b.pts[2].y / scale);
    bbox.pts[3] =
        cv::Point2f(range_col.start + b.pts[3].x / scale, range_row.start + b.pts[3].y / scale);
  }
}
}  // namespace rm_detect
