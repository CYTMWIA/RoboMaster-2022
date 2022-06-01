#ifndef __THREADING_SHARED_VARIABLE_HPP__
#define __THREADING_SHARED_VARIABLE_HPP__

#include <algorithm>
#include <condition_variable>
#include <shared_mutex>

namespace rm_threading
{
const int MAX_THREADS = 8;

template <typename DataType>
class SharedVariable
{
  /* 模板类声明与实现写在一起
   * https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
   * 之前为了加速编译时，声明与实现是分离的
   * 导致每种类型的SharedVariable都需要手动声明，且在编写cmake文件时需要增加许多额外依赖
   * 所以最后为了方便，牺牲了编译时间，将声明与实现写在一起
   */
 public:
  SharedVariable()
  {
    ptr_input_ = &(data_[0]);
    ptr_output_ = &(data_[1]);
    std::fill(updated_, updated_ + MAX_THREADS, false);
  };

  template <typename T = DataType>
  void set(T &&data)
  {
    // 因为仅有一个写者，所以在写区时无需额外加锁
    *ptr_input_ = std::forward<T>(data);

    // 读写区互换
    std::unique_lock<std::shared_mutex> xlck(mu_read_);  // 排它锁，其它线程不可读写
    std::swap(ptr_output_, ptr_input_);
    std::fill(updated_, updated_ + thread_count_, true);
    xlck.unlock();
    // 更新通知
    cv_update_.notify_all();
  };

  DataType get(bool allow_old_data = false)
  {
    int tid = get_this_thread_idx();

    std::shared_lock<std::shared_mutex> slck(mu_read_);
    if (!allow_old_data && !updated_[tid]) cv_update_.wait(slck, [&] { return updated_[tid]; });

    updated_[tid] = false;
    return *ptr_output_;
  };

  bool updated() { return updated_[get_this_thread_idx()]; }

 private:
  DataType *ptr_output_;
  DataType *ptr_input_;
  DataType data_[2];
  std::shared_mutex mu_read_;

  std::condition_variable_any cv_update_;

  uint8_t thread_count_ = 0;
  std::thread::id thread_list_[MAX_THREADS];
  bool updated_[MAX_THREADS];

  int get_this_thread_idx()
  {
    /* 这个函数看起来可以采用分离式编译，毕竟没看到有模板内容，但实际是不行的
     * 因为在分离时，该函数在实现部分中的写法如下
     *   template<class DataType>
     *   int SharedVariable<DataType>::get_this_thread_idx() { ... }
     * 所以实际上模板还是存在的。不采用分离式编译的详细理由可见此class头部的注释
     */

    auto this_id = std::this_thread::get_id();
    // 在已有线程id中查找
    for (int i = 0; i < thread_count_; i++)
    {
      if (thread_list_[i] == this_id) return i;
    }
    // 找不到则添加
    thread_list_[thread_count_] = this_id;
    thread_count_ = (thread_count_ + 1) % MAX_THREADS;
    return thread_count_ - 1;
  };
};
}  // namespace rm_threading

#endif