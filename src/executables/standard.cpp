
#include <opencv2/highgui.hpp>
#include <string>

#include "nerv/nerv.hpp"
#include "nerv/node/topic.hpp"
#include "rm_autoaim/detector_node.hpp"
#include "rm_capture/camera_node.hpp"
#include "rm_capture/video_capture_node.hpp"

int main()
{
  nerv::init("./standard.yaml");

  rm_capture::CameraNode node_cap;
  node_cap.up();

  rm_autoaim::DetectorNode node_det;
  node_det.up();

  while (true)
  {
    auto img_names = nerv::Topic<cv::Mat>::list();
    for (const auto &img_name : img_names)
    {
      if (!img_name.starts_with("debug")) continue;
      cv::imshow(img_name, nerv::Topic<cv::Mat>::get(img_name, true));
    }
  }

  return 0;
}