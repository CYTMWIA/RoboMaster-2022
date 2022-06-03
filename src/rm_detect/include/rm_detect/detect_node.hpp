#ifndef __RM_DETECT_DETECT_NODE_HPP__
#define __RM_DETECT_DETECT_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>

#include "cv_armor_detector.hpp"
#include "rm_interfaces/msg/bounding_box_array.hpp"

namespace rm_detect
{
class DetectNode : public rclcpp::Node
{
 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<rm_interfaces::msg::BoundingBoxArray>::SharedPtr pub_;
  std::thread thread_;
  CvArmorDetector detector_;

 public:
  explicit DetectNode(const std::string &input, const std::string &output,
                      const std::string &node_name = "detect_node");

 private:
  void frame_callback(sensor_msgs::msg::Image::ConstSharedPtr pframe);
};
}  // namespace rm_detect

#endif