#include "rm_autoaim/detector_thread.hpp"

#include <chrono>
#include <vector>

#include "../geometry/coordinate_converter.hpp"
#include "../geometry/pnp_solver.hpp"
#include "common/interfaces.hpp"
#include "common/logging.hpp"
#include "common/threading.hpp"
#include "common/threading/roslike_topic.hpp"
#include "power_rune_detector.hpp"

namespace rm_autoaim
{
void DetectorThread::run()
{
  PowerRuneDetectorConfig config = {.binary_thresh = 70};
  PowerRuneDetector power_rune_detector(config);

  while (true)
  {
    auto img = rm_threading::RoslikeTopic<cv::Mat>::get("capture_image");
    auto start_time = std::chrono::steady_clock::now();
    auto power_runes = power_rune_detector.detect(img);
    auto cost_time = std::chrono::steady_clock::now() - start_time;
    __LOG_DEBUG("Detect: {}ms",
                std::chrono::duration_cast<std::chrono::microseconds>(cost_time).count() / 1000.0);

    DetectResult<PowerRune> dr = {
        .image_timestamp = start_time,
        .image_robot_rpy = rmc::interfaces::Rpy{.roll = 0, .pitch = 0, .yaw = 0},
        .content = std::move(power_runes)};

    rm_threading::RoslikeTopic<decltype(dr)>::set("DetectResult/PowerRune", dr);
  }
}
}  // namespace rm_autoaim