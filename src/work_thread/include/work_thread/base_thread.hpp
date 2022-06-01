#ifndef __WORK_THREAD_BASE_THREAD_HPP__
#define __WORK_THREAD_BASE_THREAD_HPP__

#include <functional>
#include <thread>

namespace rm_work_thread
{
// class BaseThread
// {
// public:
//     virtual void up() = 0;
// };
}

/***
 * 这部分宏的目标效果（假装这是个“父类”）：
 * - 避免子类重复实现void up(void)
 * - 调用子类成员void run(void)
 * 能力不足我很抱歉，只能以如此丑陋的方式实现上述效果
 * 但把这些同质的东西直接放在每一个线程类里我认为更加丑陋
 */

#define DECL_WORKTHTREAD(name)                    \
  class name                                      \
  {                                               \
   private:                                       \
    std::thread thread_;                          \
    void run(void);                               \
                                                  \
   public:                                        \
    void up(void)                                 \
    {                                             \
      auto running = std::bind(&name::run, this); \
      thread_ = std::thread(running);             \
    }                                             \
                                                  \
   private:

#define DECL_WORKTHTREAD_END() \
  }                            \
  ;

#endif