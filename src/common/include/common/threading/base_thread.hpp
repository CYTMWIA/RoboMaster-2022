#ifndef __COMMON_THREADING_BASE_THREAD_HPP__
#define __COMMON_THREADING_BASE_THREAD_HPP__

#include <functional>
#include <thread>

namespace rm_common
{
class BaseThread
{
 private:
  std::thread thread_;

 public:
  virtual void run(void) = 0;
  void up(void)
  {
    auto running = [this]() { this->run(); };
    thread_ = std::thread(running);
  }
};
}  // namespace rm_common

#endif