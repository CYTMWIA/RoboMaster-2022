#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "capture/capture.hpp"
#include "config/config.hpp"
#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "work_thread/capture_thread.hpp"

#define IMAGE_DIR "./image/"
#define IMAGE_XML "./image_list.xml"

using namespace rmcv;
using namespace config;
using namespace threading;

int main()
{
  Config cfg;
  __LOG_INFO("读取配置文件");
  cfg.read("config.toml");

  __LOG_INFO("启动捕获线程…");
  work_thread::CaptureThread capture{cfg};
  capture.up();

  std::filesystem::remove_all(IMAGE_DIR);
  std::filesystem::create_directories(IMAGE_DIR);

  __LOG_INFO("按 C 捕获图像；按 Q 退出捕获；");

  std::vector<std::string> path_vector;
  int32_t img_count = 0;
  while (true)
  {
    cv::Mat img = RoslikeTopic<cv::Mat>::get("capture_image");

    cv::Mat show;
    cv::resize(img, show, cv::Size(800, img.rows * (800.0 / img.cols)));
    cv::imshow("CAP", show);

    auto key = cv::waitKey(10);
    if (key == 'c' || key == 'C')
    {
      __LOG_INFO("Capture {}", img_count);

      std::string path = std::string(IMAGE_DIR) + std::to_string(img_count) + std::string(".png");

      cv::imwrite(path, img);
      path_vector.push_back(path);

      img_count++;
    }
    else if (key == 'q' || key == 'Q')
    {
      __LOG_INFO("退出...");
      break;
    }
  }

  // 生成xml
  std::ofstream xml_stream(IMAGE_XML, std::ios::out);
  xml_stream << "<?xml version=\"1.0\"?><opencv_storage><images>\n";
  for (auto& path : path_vector) xml_stream << path << "\n";
  xml_stream << "</images></opencv_storage>";

  return 0;
}