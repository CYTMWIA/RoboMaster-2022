#include "communicate/serial.hpp"

#include <iostream>

#include "common/logging.hpp"
#include "serial/serial.h"

namespace rm_communicate
{
class Serial::Impl
{
 public:
  serial::Serial serial_;
  std::vector<uint8_t> recv_buffer_;

  Impl(std::string dev, uint32_t baud_rate) : serial_(dev, baud_rate), recv_buffer_(CMD_TO_CV_SIZE)
  {
  }

  void send(const CvStatus &status)
  {
    uint8_t data[CMD_TO_EC_SIZE];
    CmdToEc_make(&status, data);
    serial_.write(data, CMD_TO_EC_SIZE);
  }

  void update(RobotStatus &status)
  {
    auto new_bytes_count = serial_.available();
    if (new_bytes_count == 0) return;

    serial_.read(recv_buffer_, new_bytes_count);

    // std::cout << new_bytes_count << ": ";
    // for (const auto &c: recv_buffer_) std::cout << c << " | ";
    // std::cout << std::endl;

    auto err = CmdToCv_parse(&status, recv_buffer_.data(), recv_buffer_.size());
    if (err != 0)
    {
      __LOG_WARNING("串口数据解析失败");
    }
    else
    {
      recv_buffer_.clear();
    }
  }
};

Serial::Serial(std::string dev, uint32_t baud_rate) : pimpl{std::make_unique<Impl>(dev, baud_rate)}
{
}
Serial::~Serial() = default;

void Serial::send(const CvStatus &status) { pimpl->send(status); }

void Serial::update(RobotStatus &status) { pimpl->update(status); }
}  // namespace rm_communicate