#ifndef __RM_SERIAL_SERIAL_NODE_HPP__
#define __RM_SERIAL_SERIAL_NODE_HPP__

#include <io_context/io_context.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>
#include <thread>

#include "rm_interfaces/msg/cmd_to_ec.hpp"
#include "rm_interfaces/msg/rpy_stamped.hpp"

namespace rm_serial
{
class SerialNode : public rclcpp::Node
{
 public:
  explicit SerialNode(const rclcpp::NodeOptions& options);

 private:
  drivers::common::IoContext ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> ptr_serial_port_config_;
  drivers::serial_driver::SerialDriver serial_driver_;

  rclcpp::Subscription<rm_interfaces::msg::CmdToEc>::SharedPtr cmd_to_ec_sub_;
  rclcpp::Publisher<rm_interfaces::msg::RpyStamped>::SharedPtr gimbal_rpy_pub_;
  std::thread thread_;

  void get_parameters();
  void receive_thread();
  void send(const rm_interfaces::msg::CmdToEc::SharedPtr cmd_to_ec);
  void reopen_serial_port();
};

}  // namespace rm_serial

#endif