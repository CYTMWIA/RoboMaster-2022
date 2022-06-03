#ifndef __RM_AUTOAIM_DETECT_NODE_HPP__
#define __RM_AUTOAIM_DETECT_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>

#include "rm_interfaces/msg/bounding_boxes_with_image_size.hpp"

namespace rm_autoaim
{
class DetectNode : public rclcpp::Node
{
 private:
  class Impl;
  std::unique_ptr<Impl> pimpl;

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<rm_interfaces::msg::BoundingBoxesWithImageSize>::SharedPtr pub_;
  std::thread thread_;

 public:
  explicit DetectNode(const std::string &input, const std::string &output,
                      const std::string &node_name = "detect_node");
  ~DetectNode();

 private:
  void frame_callback(sensor_msgs::msg::Image::ConstSharedPtr pframe);
};
}  // namespace rm_autoaim

#endif