#include "rm_detect/cv_util.hpp"

#include <algorithm>

#include "rm_common/logging.hpp"
#include "rm_common/threading.hpp"

namespace rm_detect
{
RRect::RRect(const cv::RotatedRect &cvrrect)
{
  // 确定中心
  center = cvrrect.center;

  cv::Point2f pts[4];
  cvrrect.points(pts);

  // 确定长宽
  float len01 = sqrt((pts[0].x - pts[1].x) * (pts[0].x - pts[1].x) +
                     (pts[0].y - pts[1].y) * (pts[0].y - pts[1].y));
  float len03 = sqrt((pts[0].x - pts[3].x) * (pts[0].x - pts[3].x) +
                     (pts[0].y - pts[3].y) * (pts[0].y - pts[3].y));
  if (len01 < len03)
  {
    std::swap(pts[1], pts[3]);
    std::swap(len01, len03);
  }
  width = len01;
  height = len03;

  // 确定倾斜弧度
  if (pts[0].y > pts[1].y) std::swap(pts[0], pts[1]);
  rad = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
}

cv::Mat awakenlion_threshold(const cv::Mat &src, double gray_max_, double gray_avg_, double ch0,
                             double ch1, double ch2)
{
  // AWAKENLION 二值化
  // https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource
  std::vector<cv::Mat> chs;
  cv::split(src, chs);
  cv::Mat img = chs[0] * ch0 + chs[1] * ch1 + chs[2] * ch2;

  //最大灰度
  uchar gray_max = img.data[0];
  for (int i = 0; i < img.rows; i++)
  {
    uchar *p = img.data + i * img.cols;
    for (int j = 0; j < img.cols; j++)
    {
      if (*(p + j) > gray_max) gray_max = *(p + j);
    }
  }
  // 平均灰度
  long sum_gray = 0;
  for (int j = 0; j < img.rows; j++)
  {
    uchar *data = img.ptr<uchar>(j);
    for (int i = 0; i < img.cols; i++)
    {
      sum_gray += data[i];
    }
  }
  uint8_t gray_avg = sum_gray * 1.0 / (img.cols * img.rows);

  int thresh = gray_max * gray_max_ + gray_avg * gray_avg_;  // 醒师代码中两者的比例为0.6、0.4
  // std::cout << "TH: " << thresh << std::endl;

  // 二值化
  cv::Mat res;
  cv::threshold(img, res, thresh, 255, cv::THRESH_BINARY);
  return res;
}

std::vector<std::vector<cv::Point>> find_external_contours(const cv::Mat &src)
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;  // unused
  cv::findContours(src, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  return contours;
}

cv::Mat safe_range_roi(const cv::Mat &src, cv::Range range_rows, cv::Range range_cols)
{
  range_rows.start = std::max(0, range_rows.start);
  range_rows.end = std::min(src.rows, range_rows.end);
  range_cols.start = std::max(0, range_cols.start);
  range_cols.end = std::min(src.cols, range_cols.end);
  return src(range_rows, range_cols);
}

BoundingBox make_boundingbox(const RRect &left, const RRect &right)
{
  BoundingBox bbox;

  float dx = left.width / 2 * cos(left.rad), dy = left.width / 2 * sin(left.rad);
  bbox.pts[0] = cv::Point2f(left.center.x - dx, left.center.y - dy);
  bbox.pts[1] = cv::Point2f(left.center.x + dx, left.center.y + dy);

  dx = right.width / 2 * cos(right.rad), dy = right.width / 2 * sin(right.rad);
  bbox.pts[3] = cv::Point2f(right.center.x - dx, right.center.y - dy);
  bbox.pts[2] = cv::Point2f(right.center.x + dx, right.center.y + dy);

  return bbox;
}

BoundingBox make_boundingbox(const RRect &rrect)
{
  float dx = rrect.height / 2 * cos(rrect.rad), dy = rrect.height / 2 * sin(rrect.rad);
  RRect lr = rrect;
  lr.center = cv::Point2f(rrect.center.x - dx, rrect.center.y - dy);
  RRect rr = rrect;
  rr.center = cv::Point2f(rrect.center.x + dx, rrect.center.y + dy);

  return make_boundingbox(lr, rr);
}
}  // namespace rm_detect