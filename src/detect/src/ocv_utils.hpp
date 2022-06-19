#ifndef __OCV_UTILS_HPP__
#define __OCV_UTILS_HPP__

#include <opencv2/opencv.hpp>

#include "common/assert.hpp"
namespace rm_detect
{
void draw_min_area_rect(cv::Mat &img, const std::vector<cv::Point> &con, const cv::Scalar &color,
                        int thickness = 1)
{
  auto rect = cv::minAreaRect(con);
  cv::Point2f pts[4];
  rect.points(pts);
  for (size_t i = 0; i < 4; i++) cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness);
}

void threshold_for_low_exposure(const cv::Mat &img)
{
  // 输入灰度图
  RM_ASSERT("单通道图片", img.channels() == 1);

  // 灰度曲线
  double gray_curve[256];
  for (int i = 0; i < 256; i++) gray_curve[i] = 0;
  for (int i = 0; i < img.rows; i++)
    for (int j = 0; j < img.cols; j++) gray_curve[*(img.data + i * img.cols + j)] += 1;

  for (int i = 0; i < 256; i++) gray_curve[i] = (gray_curve[i] * 100) / (img.cols * img.rows);
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

  cv::threshold(img, img, thresh, 255, cv::THRESH_BINARY);
}
}  // namespace rm_detect

#endif