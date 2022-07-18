#ifndef __NERV_NODE_TOPIC_HPP__
#define __NERV_NODE_TOPIC_HPP__

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nerv/logging.hpp"

namespace nerv
{
template <typename DataType>
class Topic
{
 private:
  using slck_t = std::shared_lock<std::shared_mutex>;
  using xlck_t = std::unique_lock<std::shared_mutex>;

  class DataContainer
  {
   private:
    std::unordered_map<std::thread::id, bool> taken_;
    std::shared_ptr<DataType> pdata_;
    std::shared_mutex access_mu_;
    std::condition_variable_any cv_update_;
    bool set_;

   public:
    DataContainer() : set_(false) {}
    template <typename T>
    void set(T &&data)
    {
      {
        xlck_t xlck(access_mu_);
        pdata_ = std::make_shared<DataType>(std::forward<T>(data));
        for (auto &t : taken_) t.second = false;
      }
      set_ = true;
      cv_update_.notify_all();
    }
    DataType get(bool allow_old_data)
    {
      auto tid = std::this_thread::get_id();
      xlck_t xlck(access_mu_);
      taken_.try_emplace(tid, false);
      if ((!allow_old_data && taken_[tid]) || !set_)
        cv_update_.wait(xlck, [&] { return set_ && !taken_[tid]; });
      taken_[tid] = true;
      return *pdata_;
    }
  };

  std::unordered_map<std::string, std::shared_ptr<DataContainer>> data_map_;
  std::shared_mutex data_map_mu_;

  static inline Topic &self()
  {
    static Topic instance;
    return instance;
  }

  static void add_topic(const std::string &name)
  {
    // if (exist(name)) throw std::runtime_error("Topic 已存在");
    std::shared_ptr<DataContainer> pdc(new DataContainer());
    xlck_t xlck(self().data_map_mu_);  // 排他锁
    self().data_map_[name] = pdc;
  }

 public:
  static bool exist(const std::string &name)
  {
    slck_t lck(self().data_map_mu_);
    return self().data_map_.contains(name);
  };

  static std::vector<std::string> list()
  {
    slck_t lck(self().data_map_mu_);
    std::vector<std::string> res;
    for (const auto &pair : self().data_map_) res.push_back(pair.first);
    return res;
  }

  template <typename T>
  static void set(const std::string &name, T &&var)
  {
    if (!exist(name)) add_topic(name);
    slck_t slck(self().data_map_mu_);
    self().data_map_[name]->set(std::forward<T>(var));
  }

  static DataType get(std::string name, bool allow_old_data = false)
  {
    if (!exist(name)) add_topic(name);
    slck_t slck(self().data_map_mu_);
    return self().data_map_[name]->get(allow_old_data);
  }
};

}  // namespace nerv

#endif