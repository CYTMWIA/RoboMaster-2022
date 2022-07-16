#ifndef __COMMON_THREADING_SAFE_UNORDERED_MAP_HPP__
#define __COMMON_THREADING_SAFE_UNORDERED_MAP_HPP__

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace rm_common
{

template <typename Key, typename Tp>
class SafeUnorderedMap
{
  // 这里原计划是实现把 STL 中的 unordered_map 包装成无锁的
  // 但由于缺少满意的实现以及工期原因，暂且用锁来保证线程安全
  // 参考文章：http://kaiyuan.me/2017/12/14/lock-free-prog1/
  //           https://zhuanlan.zhihu.com/p/24983412
  // 原子 std::shared_ptr 与 std::weak_ptr 在 GCC libstdc++ 12 中被实现
  // TODO: 1. 使用 std::atomic<std::shared_ptr<unordered_map>>
  //       2. 各成员函数中对 map_ 的保护改用无锁的那一套
 private:
  using unordered_map = std::unordered_map<Key, Tp>;
  unordered_map map_;
  std::shared_mutex smu_;

// 排他锁
#define X_LOCK() std::unique_lock<std::shared_mutex> xlck(smu_)
// 共享锁
#define S_LOCK() std::shared_lock<std::shared_mutex> slck(smu_)
 public:
  std::vector<Key> keys()
  {
    std::vector<Key> keys;
    S_LOCK();
    for (const auto& kv : map_) keys.push_back(kv.first);
    return keys;
  }

  bool contains(const Key& key)
  {
    S_LOCK();
    return map_.contains(key);
  }

  Tp at(const Key& key)
  {
    S_LOCK();
    return map_.at(key);
  }

  void update(const Key& key, const Tp& val)
  {
    X_LOCK();
    map_.insert_or_assign(key);
    map_[key] = val;
  }
#undef X_LOCK
#undef S_LOCK
};

}  // namespace rm_common

#endif