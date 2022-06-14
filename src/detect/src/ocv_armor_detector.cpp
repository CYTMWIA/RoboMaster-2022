#include "detect/ocv_armor_detector.hpp"

#include "armor_icon_classifier.hpp"
#include "common/logging.hpp"
#include "common/threading.hpp"
#include "ocv_utils.hpp"
namespace rm_detect
{
/**
 * @brief 灯条
 *
 */
struct Lightbar
{
  cv::Point2f top, bottom, center;
  double rad;
  double length;

  Lightbar() = delete;
  Lightbar(const RRect &rrect)
  {
    float dx = rrect.width / 2 * cos(rrect.rad), dy = rrect.width / 2 * sin(rrect.rad);
    top = cv::Point2f(rrect.center.x - dx, rrect.center.y - dy);
    bottom = cv::Point2f(rrect.center.x + dx, rrect.center.y + dy);

    center = rrect.center;
    rad = rrect.rad;
    length = rrect.width;
  }
};

/**
 * @brief 灯条匹配结果
 *
 */
struct LightbarMatchResult
{
  int left_idx, right_idx;              // 左右灯条序号
  double confidence;                    // 置信度
  rm_data::ArmorType guess_armor_type;  // 猜测的装甲板类型
};

class OcvArmorDetector::Impl
{
 public:
  ArmorIconClassifier icon_classifier_;
  Impl(const OcvArmorDetectorSettings &settings) : icon_classifier_{settings.icon_model_path}
  {
    __LOG_INFO("装甲板检测器初始化");
  }

  void threshold(cv::Mat &img)
  {
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

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

  void match_lightbars(std::vector<Lightbar> &lightbars, std::vector<LightbarMatchResult> &results)
  {
    if (lightbars.empty()) return;

    std::sort(lightbars.begin(), lightbars.end(), [](auto &r1, auto &r2) {
      return r1.center.x < r2.center.x;
    });  // 按照中心x升序（从左到右）

    for (int i = 0; i < lightbars.size() - 1; i++)
      for (int j = i + 1; j < lightbars.size(); j++)
      {
        LightbarMatchResult res = {.left_idx = i, .right_idx = j};
        auto &left = lightbars[i];
        auto &right = lightbars[j];

        // 参考：
        // RoboMaster创梦之翼战队自瞄系统设计
        // https://zhuanlan.zhihu.com/p/416449365
        // 陈君的armor_detector
        // https://github.com/chenjunnn/rm_auto_aim/tree/main/armor_detector
        using namespace std;
        double tmp_conf, total_score = 0;
#define ADD_JUDGE(weight, min_rate, x) \
  total_score += weight;               \
  tmp_conf = (double)(weight) * (x);   \
  if (tmp_conf < weight * min_rate)    \
  {                                    \
    continue;                          \
  }                                    \
  else                                 \
    res.confidence += tmp_conf;

        // 两灯条角度差
        ADD_JUDGE(300, 0.7, 1 - abs(left.rad - right.rad) / (M_PI / 2.0));

        // 两灯条长度比应在一定范围内
        ADD_JUDGE(100, 0.5, min(left.length, right.length) / max(left.length, right.length));

        // 灯条中心距与灯条均长，顺便猜测装甲板类型
        double ratio = cv::norm(left.center - right.center) / ((left.length + right.length) / 2.0);
        if (5.0 < ratio)
          continue;
        else if (3.2 < ratio)
          res.guess_armor_type = rm_data::ARMOR_BIG;
        else if (0.8 < ratio)
          res.guess_armor_type = rm_data::ARMOR_SMALL;
        else
          continue;

#undef ADD_JUDGE
        res.confidence = res.confidence / total_score;
        std::cout << "CONF: " << res.confidence << std::endl;

        results.push_back(std::move(res));
      }
    if (results.empty()) return;

    std::sort(results.begin(), results.end(),
              [](auto &p1, auto &p2) { return p2.confidence < p1.confidence; });  // 按置信度倒序
  }

  cv::Mat extract_icon(const cv::Mat &src, const Lightbar &left, const Lightbar &right,
                       const LightbarMatchResult &match)
  {
    const cv::Size roi_size = cv::Size(20, 28);
    const int lightbar_length = 12;
    const int armor_small_width = 30;
    const int armor_big_width = 54;
    const int warp_lightbar_top_y = (roi_size.height - lightbar_length) / 2;
    const int warp_lightbar_bottom_y = warp_lightbar_top_y + lightbar_length;

    int armor_width = armor_small_width;
    if (match.guess_armor_type == rm_data::ARMOR_BIG) armor_width = armor_big_width;
    int warp_lightbar_left_x = -(armor_width - roi_size.width) / 2;
    int warp_lightbar_right_x = warp_lightbar_left_x + armor_width;

    std::vector<cv::Point2f> icon_vertexs_src = {left.top, left.bottom, right.bottom, right.top};

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

  std::vector<Armor> detect(const cv::Mat &src)
  {
    cv::Mat img;
    src.copyTo(img);
    //   double scale = 640.0 / src.cols;
    //   cv::resize(src, img, cv::Size(640.0, src.rows * scale));

    threshold(img);
    rm_threading::RoslikeTopic<cv::Mat>::set("debug_img_1", img);

    auto cons = find_external_contours(img);
    cons.erase(std::remove_if(
                   cons.begin(), cons.end(),
                   [this](const std::vector<cv::Point> &con) { return cv::contourArea(con) < 16; }),
               cons.end());

    std::vector<RRect> rrects;
    std::transform(cons.begin(), cons.end(), std::back_inserter(rrects),
                   [](const std::vector<cv::Point> &con) { return RRect{cv::minAreaRect(con)}; });
    rrects.erase(std::remove_if(rrects.begin(), rrects.end(),
                                [this](const RRect &rr) {
                                  // 倾斜度
                                  if (!(M_PI / 4.0 < rr.rad && rr.rad < M_PI * 3.0 / 4.0))
                                    return true;
                                  // 比例
                                  double ratio = rr.width / rr.height;
                                  if (!(1.5 < ratio)) return true;
                                  // 合格
                                  return false;
                                }),
                 rrects.end());
    if (rrects.empty())
    {
      return std::vector<Armor>{};
    }

    std::vector<Lightbar> lightbars;
    std::transform(rrects.begin(), rrects.end(), std::back_inserter(lightbars),
                   [](const RRect &rrect) { return Lightbar{rrect}; });
    std::vector<LightbarMatchResult> pairs;
    match_lightbars(lightbars, pairs);

    std::vector<Armor> armors;
    std::vector<uint8_t> vis(rrects.size(), 0);
    for (auto &pair : pairs)
    {
      if (pair.confidence < 0.7) break;
      if (vis[pair.left_idx] || vis[pair.right_idx]) continue;

      const Lightbar &left = lightbars[pair.left_idx];
      const Lightbar &right = lightbars[pair.right_idx];

      cv::Mat icon = extract_icon(src, left, right, pair);
      auto cres = icon_classifier_.classify(icon);
      if (cres.confidence < 0.7) continue;

      vis[pair.left_idx] = vis[pair.right_idx] = 1;

      Armor final_res = {
          .color_id = 0,
          .tag_id = cres.class_id,
          .type = pair.guess_armor_type,
      };
      final_res.pts[0] = left.top;
      final_res.pts[1] = left.bottom;
      final_res.pts[2] = right.bottom;
      final_res.pts[3] = right.top;
      armors.push_back(std::move(final_res));
    }

    return armors;
  }
};

OcvArmorDetector::OcvArmorDetector(const OcvArmorDetectorSettings &settings)
    : pimpl_{new Impl{settings}}
{
}
OcvArmorDetector::~OcvArmorDetector() = default;

std::vector<Armor> OcvArmorDetector::operator()(const cv::Mat &src) { return pimpl_->detect(src); }

}  // namespace rm_detect
