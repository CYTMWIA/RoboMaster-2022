#include "rm_autoaim/detector_node.hpp"

#include <chrono>
#include <vector>

#include "../geometry/coordinate_converter.hpp"
#include "../geometry/pnp_solver.hpp"
#include "power_rune_detector.hpp"

namespace rm_autoaim
{
DetectorNode::DetectorNode() : nerv::Node("detector_node") {}

void DetectorNode::run()
{
  PowerRuneDetectorConfig config = {.binary_thresh = 70};
  PowerRuneDetector power_rune_detector(config);

  while (true)
  {
    auto img = nerv::Topic<cv::Mat>::get("capture_image");
    auto start_time = std::chrono::steady_clock::now();
    auto power_runes = power_rune_detector.detect(img);
    auto cost_time = std::chrono::steady_clock::now() - start_time;
    NERV_DEBUG("Detect: {}ms",
               std::chrono::duration_cast<std::chrono::microseconds>(cost_time).count() / 1000.0);

    DetectResult<PowerRune> dr = {
        .image_timestamp = start_time,
        .image_robot_rpy = nerv::interfaces::Rpy{.roll = 0, .pitch = 0, .yaw = 0},
        .content = std::move(power_runes)};

    nerv::Topic<decltype(dr)>::set("DetectResult/PowerRune", dr);
  }
}
}  // namespace rm_autoaim