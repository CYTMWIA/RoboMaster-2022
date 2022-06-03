#ifndef __RM_AUTOAIM_PREDICT_NODE_HPP__
#define __RM_AUTOAIM_PREDICT_NODE_HPP__

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "rm_interfaces/msg/angle3.hpp"
#include "rm_interfaces/msg/bounding_boxes_with_image_size.hpp"

namespace rm_autoaim
{
struct PredictNodeSettings
{
  std::string input;
  std::string output;
  std::string camera_calibration_file;
};

class PredictNode : public rclcpp::Node
{
 private:
  class Impl;
  std::unique_ptr<Impl> pimpl;

 private:
  rclcpp::Subscription<rm_interfaces::msg::BoundingBoxesWithImageSize>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_debug_target_point_;
  rclcpp::Publisher<rm_interfaces::msg::Angle3>::SharedPtr pub_;
  std::thread thread_;

 public:
  explicit PredictNode(const PredictNodeSettings& settings,
                       const std::string& node_name = "predict_node");
  ~PredictNode();

 private:
  void predict(rm_interfaces::msg::BoundingBoxesWithImageSize::ConstSharedPtr pbboxes);
};
}  // namespace rm_autoaim

#endif