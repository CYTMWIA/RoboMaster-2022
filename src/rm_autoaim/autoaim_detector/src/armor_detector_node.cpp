#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <functional>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

#include "armor_detector.hpp"
#include "autoaim_interfaces/msg/targets.hpp"
namespace rm_autoaim
{

class ArmorDetectorNode : public rclcpp::Node
{
 private:
  // Camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  // Input frame
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_sub_;
  rclcpp::Publisher<autoaim_interfaces::msg::Targets>::SharedPtr targets_pub_;

  // Debug
  bool enable_debug_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_binary_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_result_img_pub_;

  // Detector
  ArmorDetector detector_;
  int binary_threshold_;
  ArmorDetector::LightbarParams lightbar_params_;
  ArmorDetector::LightbarPairParams lightbar_pair_params_;
  double icon_threshold_;

  // Functions
  void frame_callback(sensor_msgs::msg::Image::ConstSharedPtr frame_ptr);

 public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions& options);
};

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
    : Node{"armor_detector_node", options}
{
  RCLCPP_INFO(get_logger(), "Start armor_detector_node");

  // 参数
  detector_.init_armor_icon_classifier(this->declare_parameter<std::string>("icon_model_path", ""));
  icon_threshold_ = this->declare_parameter<double>("icon_threshold", 0);

  enable_debug_ = this->declare_parameter<bool>("enable_debug", true);

  binary_threshold_ = this->declare_parameter<int>("binary_threshold", 0);

  lightbar_params_.min_contour_area =
      this->declare_parameter<double>("lightbar.min_contour_area", 0);
  lightbar_params_.max_contour_area =
      this->declare_parameter<double>("lightbar.max_contour_area", INFINITY);
  lightbar_params_.min_rect_angle = this->declare_parameter<double>("lightbar.min_rect_angle", 0);
  lightbar_params_.max_rect_angle = this->declare_parameter<double>("lightbar.max_rect_angle", 180);
  lightbar_params_.min_rect_ratio = this->declare_parameter<double>("lightbar.min_rect_ratio", 0);
  lightbar_params_.max_rect_ratio =
      this->declare_parameter<double>("lightbar.max_rect_ratio", INFINITY);

  lightbar_pair_params_.min_length_ratio =
      this->declare_parameter<double>("pair.min_length_ratio", 0);
  lightbar_pair_params_.max_angle_diff =
      this->declare_parameter<double>("pair.max_angle_diff", 180);
  lightbar_pair_params_.min_center_distance_small_armor =
      this->declare_parameter<double>("pair.min_center_distance_small_armor", 0);
  lightbar_pair_params_.mid_center_distance =
      this->declare_parameter<double>("pair.mid_center_distance", 0);
  lightbar_pair_params_.max_center_distance_big_armor =
      this->declare_parameter<double>("pair.max_center_distance_big_armor", 0);

  // 订阅 & 发布
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_ptr)
      {
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info_ptr);
        cam_info_sub_.reset();  // 仅订阅一次
      });

  frame_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/frame", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::frame_callback, this, std::placeholders::_1));

  // 调试
  debug_binary_img_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("/debug_binary_img", rclcpp::SensorDataQoS());
  debug_result_img_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("/debug_result_img", rclcpp::SensorDataQoS());
}

void ArmorDetectorNode::frame_callback(sensor_msgs::msg::Image::ConstSharedPtr frame_ptr)
{
  auto start_time = this->now();
  RCLCPP_DEBUG(get_logger(), "Frame Latency: %.2lfms",
               (start_time - frame_ptr->header.stamp).seconds() * 1000);
#define LOG_TIME(label)                                 \
  RCLCPP_DEBUG(get_logger(), label " - total: %.2lfms", \
               (this->now() - start_time).seconds() * 1000);
  cv::Mat src = cv_bridge::toCvShare(frame_ptr)->image;
  LOG_TIME("toCvShare");
  //   cv::resize(src, src, cv::Size((int)src.cols * 0.5, (int)src.rows * 0.5));
  //   LOG_TIME("resize");
  auto bin = detector_.to_binary_image(src, 100);
  LOG_TIME("to_binary_image");
  auto lightbars = detector_.find_lightbars(bin, lightbar_params_);
  LOG_TIME("find_lightbars");
  auto lightbar_pairs = detector_.match_lightbars(lightbars, lightbar_pair_params_);
  LOG_TIME("match_lightbars");
  detector_.filter_by_icon(src, lightbar_pairs, icon_threshold_);
  LOG_TIME("filter_by_icon");
  detector_.filter_by_color(src, lightbar_pairs);
  LOG_TIME("filter_by_color");

#undef LOG_TIME
  if (enable_debug_)
  {
    auto bin_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", bin).toImageMsg();
    bin_msg->header.stamp = start_time;
    debug_binary_img_pub_->publish(*bin_msg);

    cv::Mat res(src);
    detector_.draw_result_image(res, lightbar_pairs);
    auto result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", res).toImageMsg();
    result_msg->header.stamp = start_time;
    debug_result_img_pub_->publish(*result_msg);
  }
}

}  // namespace rm_autoaim

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(rm_autoaim::ArmorDetectorNode)