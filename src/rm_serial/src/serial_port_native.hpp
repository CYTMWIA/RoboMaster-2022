#pragma once

// C++
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rm_serial
{
class SerialPort
{
 private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;

 public:
  SerialPort();
  ~SerialPort();

  void open(const std::string& dev, uint32_t baud_rate);
  void reopen();

  uint32_t available();

  void read(uint8_t* pdata, size_t size);
  void write(uint8_t* pdata, size_t size);

  // 可能有用，但现在没用
  // void read(std::vector<uint8_t>& buffer);
  // void write(const std::vector<uint8_t>& data);

  // TODO
  // void async_read();
  // void async_write();
  // void epoll();
};
}  // namespace rm_serial
