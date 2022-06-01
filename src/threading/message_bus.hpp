#ifndef __THREADING_MESSAGE_BUS_HPP__
#define __THREADING_MESSAGE_BUS_HPP__

#include <cstring>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <thread>
#include <unordered_map>

namespace rmcv::threading
{
enum class ThreadCode
{
  main = 0,
  capture,
  communicate,
  detect,
  record,
  strategy,
  web,
};

const int32_t MAX_THREADS = 8;

class MessageContent : public std::vector<uint8_t>
{
  /* 此class用于线程间通信、命令
   * 由于不涉及跨设备通信，所以不考虑大小端
   */
 public:
  template <typename T>
  void write(T&& val)
  {
    int count = size();
    resize(size() + sizeof(T));
    memcpy(data() + count, &val, sizeof(T));
  }
  template <typename T>
  T read()
  {
    if (read_pos_ >= size()) throw std::runtime_error("MessageContent Read() 越界");

    T res;
    memcpy(&res, data() + read_pos_, sizeof(T));
    read_pos_ += sizeof(T);
    return res;
  }

 private:
  int32_t read_pos_ = 0;
};

struct Message
{
  Message(MessageContent content_, ThreadCode from_);

  ThreadCode from;
  MessageContent content;
};

class MessageBus
{
  /* 作为消息（命令）总线，该class使用单例模式
   * 使用connect来“连接”到总线
   */
 public:
  static MessageBus& connect();

  void checkin(ThreadCode code);  // 在发送或接收消息之前，线程必须先在此“登记”自己的身份
  bool is_checkin();
  ThreadCode get_code();

  void send_message(ThreadCode to, MessageContent content);
  bool has_message();
  // void wait_message_for(const std::chrono::duration<Rep, Period>& duration);
  Message receive_message();

 private:
  MessageBus(){};

  std::unordered_map<std::thread::id, ThreadCode> to_code_;
  std::shared_mutex to_code_mu_;

  std::queue<Message> msgbox_[MAX_THREADS];
  std::shared_mutex msgbox_mu_[MAX_THREADS];
};
}  // namespace rmcv::threading

#endif