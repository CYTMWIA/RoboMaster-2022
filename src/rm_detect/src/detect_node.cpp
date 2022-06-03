#include "rm_detect/detect_node.hpp"

#include "rm_common/image_msg.hpp"
#include "rm_common/logging.hpp"
namespace rm_detect
{
DetectNode::DetectNode(const std::string &input, const std::string &output,
                       const std::string &node_name)
    : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  pub_ = this->create_publisher<rm_interfaces::msg::BoundingBoxArray>(output, rclcpp::SensorDataQoS());
  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input, rclcpp::SensorDataQoS(),
      std::bind(&DetectNode::frame_callback, this, std::placeholders::_1));
}

void DetectNode::frame_callback(sensor_msgs::msg::Image::ConstSharedPtr pframe)
{
  cv::Mat frame = rm_common::msg2image(pframe);
  std::vector<rm_detect::BoundingBox> dets;
  if (frame.empty())
  {
    __LOG_WARNING("获取图像为空");
  }else
  {
    dets = detector_(frame);
  }
 
  rm_interfaces::msg::BoundingBoxArray bba;
  for (const auto &ob : dets)
  {
    rm_interfaces::msg::BoundingBox bb;
    bb.color_id = ob.color_id;
    bb.confidence = ob.confidence;
    bb.tag_id = ob.tag_id;
    for (int i = 0; i < 4; i++)
    {
      bb.points[i].x = ob.pts[i].x;
      bb.points[i].y = ob.pts[i].y;
    }

    bba.boundingboxes.push_back(std::move(bb));
  }

  pub_->publish(std::move(bba));
}

}  // namespace rm_detect