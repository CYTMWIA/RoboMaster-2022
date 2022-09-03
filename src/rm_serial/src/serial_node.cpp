#include "rm_serial/serial_node.hpp"

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/concept_check.hpp>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "cmd.h"
#include "nerv/nerv.hpp"

namespace rm_serial
{

struct SerialNode::Impl
{
  std::string port;
  int32_t baud_rate;
  int32_t frequency;

  // 暂时没有用两个线程分别负责读/写的需求
  // 而且两个线程对于同一串口的共用可能不太好写
  // std::thread recv_thread_;

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  boost::asio::steady_timer write_timer_;

  static const int read_buffer_size_ = CMD_TO_CV_SIZE * 2;
  int read_buffer_idx_ = 0;
  std::array<uint8_t, read_buffer_size_> read_buffer_;

  Impl() : serial_port_(io_context_), write_timer_(io_context_) {}

  void open_serial_port()
  {
    while (true)
    {
      try
      {
        serial_port_.open(port);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        return;
      }
      catch (const boost::system::system_error& err)
      {
        NERV_ERROR("打开串口失败 {}: {}", port, err.what());
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  void read_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
  {
    boost::ignore_unused_variable_warning(error);
    if (bytes_transferred > 0)
    {
    }
  }

  void write_handler(const boost::system::error_code& error)
  {
    boost::ignore_unused_variable_warning(error);
    // Next
    write_timer_.expires_at(write_timer_.expires_at() + std::chrono::seconds(1) / frequency);
    write_timer_.async_wait(std::bind(&Impl::write_handler, this, std::placeholders::_1));
  }
};
SerialNode::~SerialNode() = default;

SerialNode::SerialNode() : nerv::Node("serial_node"), pimpl_(new Impl())
{
  bool invalid_argument = false;
  pimpl_->port = this->get_parameter<std::string>("port", "");
  pimpl_->baud_rate = this->get_parameter<int32_t>("baud_rate", 115200);
  pimpl_->frequency = this->get_parameter<int32_t>("frequency", 500);

  if (pimpl_->port == "")
  {
    NERV_ERROR("参数错误 port");
    invalid_argument = true;
  }
  if (invalid_argument) throw std::invalid_argument("");
}

void SerialNode::run()
{
  pimpl_->open_serial_port();

  // 写
  pimpl_->write_timer_.expires_after(std::chrono::seconds(1) / pimpl_->frequency);
  pimpl_->write_timer_.async_wait(
      std::bind(&Impl::write_handler, pimpl_.get(), std::placeholders::_1));
  // 读
  pimpl_->serial_port_.async_read_some(
      boost::asio::buffer(pimpl_->read_buffer_),
      std::bind(&Impl::read_handler, pimpl_.get(), std::placeholders::_1, std::placeholders::_2));

  pimpl_->io_context_.run();
}

}  // namespace rm_serial