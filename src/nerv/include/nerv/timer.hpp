#ifndef __NERV_TIMER_HPP__
#define __NERV_TIMER_HPP__

#include <fmt/format.h>

#include <chrono>
#include <string>

#include "nerv/logging.hpp"
namespace nerv
{

class Timer
{
 private:
  std::invoke_result<decltype(std::chrono::steady_clock::now)>::type start_time_;
  decltype(start_time_) end_time_;

 public:
  inline auto start() { return start_time_ = std::chrono::steady_clock::now(); }
  inline auto end()
  {
    end_time_ = std::chrono::steady_clock::now();
    return end_time_ - start_time_;
  }
  inline auto cost_time() { return end_time_ - start_time_; }
  inline auto cost_time_ms()
  {
    auto us = (std::chrono::duration_cast<std::chrono::microseconds>(cost_time())).count();
    return us / 1000.0;
  }
};

class ScopedTimer
{
 private:
  Timer timer_;
  std::string label_;

 public:
  inline ScopedTimer(const std::string& label) : label_(label) { timer_.start(); };
  inline ~ScopedTimer()
  {
    timer_.end();
    NERV_INFO("{} cost time: {:.3f}ms", label_, timer_.cost_time_ms());
  }
};

}  // namespace nerv

#endif