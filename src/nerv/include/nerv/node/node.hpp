#ifndef __NERV_NODE_NODE_HPP__
#define __NERV_NODE_NODE_HPP__

#include <functional>
#include <thread>

#include "nerv/nerv_base.hpp"

namespace nerv
{
class Node
{
 private:
  std::string node_name_;
  std::thread thread_;

 public:
  Node() = delete;
  explicit Node(const std::string& node_name) : node_name_(node_name) {}

  template <typename T, typename... Args>
  T get_parameter(Args&&... args)
  {
    return nerv::get_parameter<T>(node_name_, args...);
  }

  virtual void run(void) = 0;
  void up(void)
  {
    auto running = [this]() { this->run(); };
    thread_ = std::thread(running);
  }
};
}  // namespace nerv

#endif