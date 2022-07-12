// Ours
#include "rm_serial/serial_node.hpp"

#include "cmd.h"

// Others
#include <boost/algorithm/string.hpp>

// ROS
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <io_context/io_context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace rm_serial
{
SerialNode::SerialNode(const rclcpp::NodeOptions& options)
    : Node{"serial_node", options}, ctx_{2}, serial_driver_(ctx_)

{
  RCLCPP_INFO(get_logger(), "Start SerialNode");

  get_parameters();

  cmd_to_ec_sub_ = this->create_subscription<rm_interfaces::msg::CmdToEc>(
      "/cmd_to_ec", rclcpp::SensorDataQoS(),
      std::bind(&SerialNode::send, this, std::placeholders::_1));

  gimbal_rpy_pub_ = this->create_publisher<rm_interfaces::msg::RpyStamped>(
      "/gimbal_rpy", rclcpp::QoS(rclcpp::KeepLast(1)));

  try
  {
    serial_driver_.init_port(device_name_, *ptr_serial_port_config_);
    if (!serial_driver_.port()->is_open())
    {
      serial_driver_.port()->open();
      thread_ = std::thread(&SerialNode::receive_thread, this);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(),
                 ex.what());
    throw ex;
  }
}

void SerialNode::receive_thread()
{
  std::vector<uint8_t> data;
  data.resize(CMD_TO_CV_SIZE);

  while (rclcpp::ok())
  {
    try
    {
      serial_driver_.port()->receive(data);

      RobotState robot_state;
      auto err = CmdToCv_parse(&robot_state, data.data(), CMD_TO_CV_SIZE);
      if (err != 0)
      {
        RCLCPP_WARN(get_logger(), "Parse serial data fail with size %d", data.size());
      }
      else
      {
        rm_interfaces::msg::RpyStamped rs;
        rs.header.stamp = this->now();
        rs.roll = 0;
        rs.pitch = robot_state.pitch;
        rs.yaw = robot_state.yaw;
        gimbal_rpy_pub_->publish(rs);
      }
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
      reopen_serial_port();
    }
  }
}

void SerialNode::send(const rm_interfaces::msg::CmdToEc::SharedPtr cmd_to_ec)
{
  CmdToEc cmd;
  cmd.fire = cmd_to_ec->fire;
  cmd.pitch = cmd_to_ec->gimbal_add.pitch;
  cmd.yaw = cmd_to_ec->gimbal_add.yaw;

  std::vector<uint8_t> data(CMD_TO_EC_SIZE);
  CmdToEc_make(&cmd, data.data());

  try
  {
    serial_driver_.port()->send(data);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopen_serial_port();
  }
}

void SerialNode::reopen_serial_port()
{
  while (rclcpp::ok())
  {
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try
    {
      if (serial_driver_.port()->is_open())
      {
        serial_driver_.port()->close();
      }
      serial_driver_.port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
      return;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
}

void SerialNode::get_parameters()
{
  device_name_ = declare_parameter<std::string>("device_name", "");

  uint32_t baud_rate = declare_parameter<int>("baud_rate", 0);

  using FlowControl = drivers::serial_driver::FlowControl;
  FlowControl flow_control;
  auto flow_control_string = declare_parameter<std::string>("flow_control", "");
  boost::algorithm::trim(flow_control_string);
  boost::algorithm::to_upper(flow_control_string);
  if (flow_control_string == "NONE")
    flow_control = FlowControl::NONE;
  else if (flow_control_string == "HARDWARE")
    flow_control = FlowControl::HARDWARE;
  else if (flow_control_string == "SOFTWARE")
    flow_control = FlowControl::SOFTWARE;
  else
    throw std::invalid_argument("Invalid parameter: flow_control");

  using Parity = drivers::serial_driver::Parity;
  Parity parity;
  auto parity_string = declare_parameter<std::string>("parity", "");
  boost::algorithm::trim(parity_string);
  boost::algorithm::to_upper(parity_string);
  if (parity_string == "NONE")
    parity = Parity::NONE;
  else if (parity_string == "EVEN")
    parity = Parity::EVEN;
  else if (parity_string == "ODD")
    parity = Parity::ODD;
  else
    throw std::invalid_argument("Invalid parameter: parity");

  using StopBits = drivers::serial_driver::StopBits;
  StopBits stop_bits;
  auto stop_bits_string = declare_parameter<std::string>("stop_bits", "");
  boost::algorithm::trim(stop_bits_string);
  boost::algorithm::to_upper(stop_bits_string);
  if ((std::set<std::string>{"1", "1.", "1.0", "ONE"}).count(stop_bits_string))
    stop_bits = StopBits::ONE;
  else if ((std::set<std::string>{"1.5", "ONE_POINT_FIVE"}).count(stop_bits_string))
    stop_bits = StopBits::ONE_POINT_FIVE;
  else if ((std::set<std::string>{"2", "2.", "2.0", "TWO"}).count(stop_bits_string))
    stop_bits = StopBits::TWO;
  else
    throw std::invalid_argument("Invalid parameter: stop_bits");

  ptr_serial_port_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
      baud_rate, flow_control, parity, stop_bits);
}
}  // namespace rm_serial

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial::SerialNode)