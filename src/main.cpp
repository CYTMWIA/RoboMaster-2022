#include <opencv2/opencv.hpp>
#include <thread>

#include "communicate/communicate.hpp"
#include "config/config.hpp"
#include "detect/detect.hpp"
#include "logging/logging.hpp"
#include "predict/predict.hpp"
#include "threading/threading.hpp"
#include "util/util.hpp"
#include "work_thread/work_thread.hpp"

using namespace rmcv;
using namespace config;
using namespace threading;
using namespace detect;
using namespace communicate;
using namespace predict;
using namespace util;

#define DEBUG_IMSHOW_WIDTH 720.0  // 调试图像宽度

void debug_imshow(const std::string name, const cv::Mat &img)
{
  cv::Mat show;
  if (img.cols > DEBUG_IMSHOW_WIDTH)
    cv::resize(img, show,
               cv::Size(DEBUG_IMSHOW_WIDTH, img.rows * (1.0 * DEBUG_IMSHOW_WIDTH / img.cols)));
  else
    show = img;
  cv::imshow(name, show);
}

int main(int, char **)
{
  Config cfg;
  __LOG_INFO("读取配置文件");
  cfg.read("config.toml");

  /**********************************
   * 启动线程
   */
  __LOG_INFO("启动捕获线程…");
  work_thread::CaptureThread capture{cfg};
  capture.up();

  __LOG_INFO("启动检测线程…");
  work_thread::DetectThread detect{cfg};
  detect.up();

  __LOG_INFO("启动通信线程…");
  work_thread::CommunicateThread communicate{cfg};
  communicate.up();

  __LOG_INFO("启动决策线程…");
  work_thread::StrategyThread strategy{cfg};
  strategy.up();

  __LOG_INFO("所有线程已启动");

  // while (true) std::this_thread::sleep_for(std::chrono::seconds(1));

  /**********************************
   * 调试用
   */

  while (true)
  {
    if (!cfg.debug.enable_window)
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    // 可视化检测结果
    auto ori_img = RoslikeTopic<cv::Mat>::get("capture_image");
    auto detections = RoslikeTopic<std::vector<BoundingBox>>::get("detect_result", true);
    cv::Mat img;  // 避免引用源图像
    ori_img.copyTo(img);
    const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
    for (const auto &b : detections)
    {
      bool break_flag = false;
      for (int i = 0; i < 4; i++)
      {
        if (b.pts[i].x != b.pts[i].x || b.pts[i].y != b.pts[i].y)  // NaN
        {
          break_flag = true;
          break;
        }
      }
      if (break_flag) break;
      cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
      cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
      cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
      cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
      cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1,
                  colors[b.color_id], 3);
    }
    debug_imshow("SHOW", img);
    if (RoslikeTopic<cv::Mat>::exist("debug_img_1"))
      debug_imshow("debug_img_1", RoslikeTopic<cv::Mat>::get("debug_img_1", true));
    if (RoslikeTopic<cv::Mat>::exist("debug_img_2"))
      debug_imshow("debug_img_2", RoslikeTopic<cv::Mat>::get("debug_img_2", true));
    cv::waitKey(5);
  }

  return 0;
}
