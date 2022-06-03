#include "rm_autoaim/detect_node.hpp"

#include "common/image_msg.hpp"
#include "detect/cv_armor_detector.hpp"
#include "rm_common/logging.hpp"

namespace rm_autoaim
{
class DetectNode::Impl
{
 public:
  CvArmorDetector detector_;
};

DetectNode::DetectNode(const std::string &input, const std::string &output,
                       const std::string &node_name)
    : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
      pimpl(std::make_unique<Impl>())
{
  pub_ = this->create_publisher<rm_interfaces::msg::BoundingBoxesWithImageSize>(
      output, rclcpp::SensorDataQoS());
  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input, rclcpp::SensorDataQoS(),
      std::bind(&DetectNode::frame_callback, this, std::placeholders::_1));
}
DetectNode::~DetectNode() = default;

void DetectNode::frame_callback(sensor_msgs::msg::Image::ConstSharedPtr pframe)
{
  cv::Mat frame = msg2image(pframe);
  std::vector<BoundingBox> dets;
  if (frame.empty())
  {
    __LOG_WARNING("获取图像为空");
  }
  else
  {
    dets = pimpl->detector_(frame);
  }

  rm_interfaces::msg::BoundingBoxesWithImageSize bbwis;
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

    bbwis.boundingboxes.push_back(std::move(bb));
  }
  bbwis.image_height = frame.rows;
  bbwis.image_width = frame.cols;

  pub_->publish(std::move(bbwis));
}

}  // namespace rm_autoaim