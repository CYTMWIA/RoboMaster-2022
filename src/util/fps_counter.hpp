#ifndef __UTIL_FPS_COUNTER_HPP__
#define __UTIL_FPS_COUNTER_HPP__

#include <chrono>

namespace rmcv::util
{
class FpsCounter
{
 private:
  std::chrono::time_point<std::chrono::steady_clock> tick_time_;
  bool first_flag_;

 public:
  FpsCounter();
  float tick();
};
}  // namespace rmcv::util

#endif