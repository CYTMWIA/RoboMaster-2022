#include "detect/hamming_match.hpp"

#include "common/threading.hpp"
#include "detect/cv_util.hpp"

namespace rm_detect
{
HammingMatch::HammingMatch(const cv::Mat &_template) : template_(_template)
{
  CV_Assert(CV_MAT_TYPE(_template.type()) == CV_8UC1);
}

void HammingMatch::set_template(const cv::Mat &_template)
{
  CV_Assert(CV_MAT_TYPE(_template.type()) == CV_8UC1);
  template_ = _template;
}

double HammingMatch::match(const cv::Mat &img) const
{
  CV_Assert(CV_MAT_TYPE(img.type()) == CV_8UC1);

  cv::Mat resized;
  cv::resize(img, resized, template_.size());

  int64_t sum = 0;
  for (int i = 0; i < template_.rows; i++)
  {
    uint8_t *p1 = template_.data + i * template_.cols;
    uint8_t *p2 = resized.data + i * template_.cols;
    for (int j = 0; j < template_.cols; j++)
    {
      sum += (p1[j] == p2[j]);
    }
  }
  return 1.0 * sum / (template_.cols * template_.rows);
}

double HammingMatch::operator()(const cv::Mat &img) const { return match(img); }

HammingMatch make_icon_hammimg_match(const cv::Mat &icon_image)
{
  cv::Mat icon_template;
  cv::cvtColor(icon_image, icon_template, cv::COLOR_BGR2GRAY);
  cv::threshold(icon_template, icon_template, 0, 255, cv::THRESH_OTSU);

  std::vector<std::vector<cv::Point>> contours = find_external_contours(icon_template);

  cv::Point2f center(icon_template.cols / 2.0, icon_template.rows / 2.0);
  for (const auto &con : contours)
  {
    auto rect = cv::boundingRect(con);
    if (rect.contains(center))
    {
      cv::Mat _template = icon_template(rect);
      cv::resize(_template, _template, cv::Size(32.0, _template.rows * (32.0 / _template.cols)));
      return HammingMatch(_template);
    }
  }

  throw std::runtime_error("创建图标汉明匹配器失败");
}

HammingMatch make_icon_hammimg_match(std::string icon_path)
{
  cv::Mat icon = cv::imread(icon_path);
  return make_icon_hammimg_match(icon);
}
}  // namespace rm_detect
