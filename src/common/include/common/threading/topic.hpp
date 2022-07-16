#ifndef __COMMON_THREADING_TOPIC_HPP__
#define __COMMON_THREADING_TOPIC_HPP__

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "safe_unordered_map.hpp"

namespace rm_common
{
template <typename Tp>
class Shipper
{
  // clang-format off
  // 仅可有一写者，否则可能发生数据竞争
  // 参考：https://zh.cppreference.com/w/cpp/memory/shared_ptr
  // > 多个线程能在 shared_ptr 的不同实例上调用所有成员函数（包含复制构造函数与复制赋值）而不附加同步， 即使这些实例是副本，且共享同一对象的所有权。
  // > 若多个执行线程访问同一 shared_ptr 而不同步，且任一线程使用 shared_ptr 的非 const 成员函数，则将出现数据竞争；
  // > 原子函数的 shared_ptr 特化能用于避免数据竞争。 
  // 为什么不适配多写者的情况？因为本项目目前只有一写多读的情况
  // clang-format on

 private:
  std::shared_ptr<const Tp> pdata_;
  SafeUnorderedMap<std::thread::id, bool> taken_;

  inline auto get_thread_id() { return std::this_thread::get_id(); }

 public:
  std::shared_ptr<const Tp> get(bool allow_taken = false)
  {
    if (!allow_taken)
    {
      auto id = get_thread_id();
      if (!taken_.contains(id)) taken_.update(id, false);
      while (taken_.at(id))
        ;
      taken_.update(id, true);
    }
    return pdata_;
  }
  void set(const Tp& val)
  {
    pdata_ = std::make_shared<const Tp>(val);
    for (const auto& id : taken_.keys()) taken_.update(id, false);
  }
};

template <typename Tp>
class Topic
{
 private:
 public:
  static Topic& connect()
  {
    static Topic<Tp> instance;
    return instance;
  }

  void publish(const std::string& name, Tp var) {}

  Tp get_latest(const std::string& name) {}
};

}  // namespace rm_common

#endif