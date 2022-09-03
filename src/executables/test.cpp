
#include <opencv2/highgui.hpp>
#include <string>

#include "nerv/nerv.hpp"
#include "nerv/node/topic.hpp"
#include "rm_capture/camera_node.hpp"
#include "rm_capture/video_capture_node.hpp"

int main()
{
  nerv::init("/home/rm/2022-REDO/build/test.yaml");

  rm_capture::CameraNode node;
  node.up();

  while (true)
  {
    cv::Mat capture_frame = nerv::Topic<cv::Mat>::get("capture_frame");
    cv::imshow("capture_frame", capture_frame);
    cv::waitKey(10);
  }

  //     NERV_INFO("{}", nerv::get_parameter<double>("aaa", "ccc", 9));
  //     // NERV_INFO("{}", nerv::get_parameter<int>("aaa", "eee", 0));
  //     NERV_INFO("{}", nerv::get_parameter<int>("aasda", "bbgfgbd", 80));
  //     // YAML::TypedBadConversion<typename T>
  // nerv::Topic<int>::set("aaa", 123);
  // nerv::Topic<int>::get("aaa");
  // // nerv::Topic<int>::set("afsdf", 123);
  //     // auto keys = nerv::Topic<int>::list();
  //     // NERV_INFO("{}", keys.size());

  return 0;
}