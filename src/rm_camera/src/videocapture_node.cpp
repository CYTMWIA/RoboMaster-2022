// ROS
#include <rmw/qos_profiles.h>

#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>

// Others
#include <boost/algorithm/string.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

namespace rm_camera
{
class VideoCaptureNode : public rclcpp::Node
{
 private:
  /**
   * @brief 捕获目标，可为整数或字符串
   * 整数：摄像头id
   * 字符串：视频路径
   */
  std::string capture_target_;

  int32_t rotate_code;

  image_transport::CameraPublisher camera_pub_;
  std::thread thread_;
  cv::VideoCapture video_capture_;

  void get_parameters()
  {
    capture_target_ = this->declare_parameter<std::string>("capture_target", "0");

    auto rotate_s = this->declare_parameter<std::string>("rotate", "0");
    boost::algorithm::trim(rotate_s);
    boost::algorithm::to_upper(rotate_s);
    if ((std::set<std::string>{"NONE", "0", "0.", "0.0"}).count(rotate_s))
      rotate_code = -1;
    else if ((std::set<std::string>{"LEFT", "90", "90.", "90.0"}).count(rotate_s))
      rotate_code = cv::ROTATE_90_COUNTERCLOCKWISE;
    else if ((std::set<std::string>{"RIGHT", "-90", "-90.", "-90.0"}).count(rotate_s))
      rotate_code = cv::ROTATE_90_CLOCKWISE;
    else if ((std::set<std::string>{"180", "180.", "180.0"}).count(rotate_s))
      rotate_code = cv::ROTATE_180;
    else
      throw std::invalid_argument("Invalid parameter: rotate");
  }

  void open_video_capture()
  {
    try
    {
      video_capture_.open(std::stoi(capture_target_));
    }
    catch (const std::invalid_argument &ex)
    {
      video_capture_.open(capture_target_);
    }

    if (!video_capture_.isOpened())
    {
      throw std::runtime_error("Open VideoCapture fail");
    }
  }

  void capture_thread()
  {
    cv::Mat frame;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    camera_info_msg_.header.frame_id = "0";
    sensor_msgs::msg::Image image_msg_;
    while (rclcpp::ok())
    {
      video_capture_ >> frame;
      if (frame.empty())
      {
        RCLCPP_ERROR(get_logger(), "Capture frame error");
        while (rclcpp::ok())
        {
          RCLCPP_WARN(get_logger(), "Attempting to reopen VideoCapture");
          try
          {
            open_video_capture();
            RCLCPP_INFO(get_logger(), "Successfully reopened VideoCapture");
            break;
          }
          catch (const std::exception &ex)
          {
            RCLCPP_ERROR(get_logger(), "Error while reopening VideoCapture: %s", ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
          }
        }
        continue;
      }
      if (rotate_code != -1) cv::rotate(frame, frame, rotate_code);

      image_msg_.header.stamp = camera_info_msg_.header.stamp = this->now();
      image_msg_.height = camera_info_msg_.height = frame.rows;
      image_msg_.width = camera_info_msg_.width = frame.cols;
      image_msg_.encoding = "bgr8";
      image_msg_.is_bigendian = false;
      image_msg_.step = frame.cols * frame.elemSize();
      size_t size = image_msg_.step * frame.rows;
      image_msg_.data.resize(size);

      if (frame.isContinuous())
      {
        memcpy(reinterpret_cast<char *>(&image_msg_.data[0]), frame.data, size);
      }
      else
      {
        // Copy by row by row
        uchar *ros_data_ptr = reinterpret_cast<uchar *>(&image_msg_.data[0]);
        uchar *cv_data_ptr = frame.data;
        for (int i = 0; i < frame.rows; ++i)
        {
          memcpy(ros_data_ptr, cv_data_ptr, image_msg_.step);
          ros_data_ptr += image_msg_.step;
          cv_data_ptr += frame.step;
        }
      }

      camera_pub_.publish(image_msg_, camera_info_msg_);
    }
  }

 public:
  explicit VideoCaptureNode(const rclcpp::NodeOptions &options) : Node("videocapture_node", options)
  {
    get_parameters();

    bool use_sensor_data_qos = this->declare_parameter<bool>("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "frame", qos);

    open_video_capture();
    thread_ = std::thread(&VideoCaptureNode::capture_thread, this);
  }
};
}  // namespace rm_camera

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(rm_camera::VideoCaptureNode)