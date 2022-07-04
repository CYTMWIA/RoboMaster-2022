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
#include <vector>

// 大华
#include <IMVApi.h>
#include <IMVDefines.h>

// Others
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

namespace rm_camera
{
class DahuaCameraNode : public rclcpp::Node
{
 private:
  IMV_HANDLE dev_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  image_transport::CameraPublisher camera_pub_;
  std::thread thread_;

  std::string get_imv_error_string(int error_code)
  {
    switch (error_code)
    {
      case IMV_OK:
        return std::string("成功，无错误");
      case IMV_ERROR:
        return std::string("通用的错误");
      case IMV_INVALID_HANDLE:
        return std::string("错误或无效的句柄");
      case IMV_INVALID_PARAM:
        return std::string("错误的参数");
      case IMV_INVALID_FRAME_HANDLE:
        return std::string("错误或无效的帧句柄");
      case IMV_INVALID_FRAME:
        return std::string("无效的帧");
      case IMV_INVALID_RESOURCE:
        return std::string("相机/事件/流等资源无效");
      case IMV_INVALID_IP:
        return std::string("设备与主机的IP网段不匹配");
      case IMV_NO_MEMORY:
        return std::string("内存不足");
      case IMV_INSUFFICIENT_MEMORY:
        return std::string("传入的内存空间不足");
      case IMV_ERROR_PROPERTY_TYPE:
        return std::string("属性类型错误");
      case IMV_INVALID_ACCESS:
        return std::string("属性不可访问、或不能读/写、或读/写失败");
      case IMV_INVALID_RANGE:
        return std::string("属性值超出范围、或者不是步长整数倍");
      case IMV_NOT_SUPPORT:
        return std::string("设备不支持的功能");
    }
    return std::string("未知错误");
  }

  int open_camera(int camera_id)
  {
    int err = IMV_OK;

    IMV_DeviceList devices;
    err = IMV_EnumDevices(&devices, interfaceTypeAll);
    if (err != IMV_OK) throw std::runtime_error("获取设备列表失败：" + get_imv_error_string(err));
    if (devices.nDevNum < 1) throw std::runtime_error("未发现设备");

    err = IMV_CreateHandle(&dev_, modeByIndex, (void *)&camera_id);
    if (err != IMV_OK) throw std::runtime_error("创建设备句柄失败：" + get_imv_error_string(err));
    err = IMV_Open(dev_);
    if (err != IMV_OK) throw std::runtime_error("打开相机失败：" + get_imv_error_string(err));

    return err;
  }

  int start_streaming()
  {
    int err = IMV_StartGrabbing(dev_);
    if (err != IMV_OK) throw std::runtime_error("拉流失败：" + get_imv_error_string(err));
    return err;
  }

  // 相机id
  int camera_id_;

  // 旋转图像参数
  int rotate_code_;
  int rotate(std::string rotate_s)
  {
    boost::algorithm::trim(rotate_s);
    boost::algorithm::to_upper(rotate_s);
    if ((std::set<std::string>{"NONE", "0", "0.", "0.0"}).count(rotate_s))
      rotate_code_ = -1;
    else if ((std::set<std::string>{"LEFT", "90", "90.", "90.0"}).count(rotate_s))
      rotate_code_ = cv::ROTATE_90_COUNTERCLOCKWISE;
    else if ((std::set<std::string>{"RIGHT", "-90", "-90.", "-90.0"}).count(rotate_s))
      rotate_code_ = cv::ROTATE_90_CLOCKWISE;
    else if ((std::set<std::string>{"180", "180.", "180.0"}).count(rotate_s))
      rotate_code_ = cv::ROTATE_180;
    else
      return IMV_INVALID_PARAM;
    return IMV_OK;
  }

#define SET_IMV(type, name, value)                                                           \
  err = IMV_Set##type##FeatureValue(dev_, (name), (value));                                  \
  if (err != IMV_OK)                                                                         \
  {                                                                                          \
    RCLCPP_WARN(get_logger(), "设置 " #name " 失败：%s", get_imv_error_string(err).c_str()); \
    return err;                                                                              \
  }

  //曝光时间
  double exposure_time_;
  int exposure_time(double value)
  {
    int err;
    SET_IMV(Double, "ExposureTime", value);
    exposure_time_ = value;
    return err;
  }

  // 增益
  double gain_;
  int gain(double value)
  {
    int err;
    SET_IMV(Double, "GainRaw", value)
    gain_ = value;
    return err;
  }

#define SET_IMV_WHITE_BALANCE(ch, val)        \
  SET_IMV(Enum, "BalanceWhiteAuto", 0)        \
  SET_IMV(Enum, "BalanceRatioSelector", (ch)) \
  SET_IMV(Double, "BalanceRatio", (val))

  // 白平衡：蓝色通道
  double white_balance_blue_;
  int white_balance_blue(double value)
  {
    int err;
    SET_IMV_WHITE_BALANCE(2, value)
    white_balance_blue_ = value;
    return err;
  }

  // 白平衡：绿色通道
  double white_balance_green_;
  int white_balance_green(double value)
  {
    int err;
    SET_IMV_WHITE_BALANCE(1, value)
    white_balance_green_ = value;
    return err;
  }

  // 白平衡：红色通道
  double white_balance_red_;
  int white_balance_red(double value)
  {
    int err;
    SET_IMV_WHITE_BALANCE(0, value)
    white_balance_red_ = value;
    return err;
  }

#undef SET_IMV_WHITE_BALANCE
#undef SET_IMV

  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters)
    {
      int err = IMV_OK;
      if (param.get_name() == "camera_id")
      {
        camera_id_ = param.as_int();
        reinit_camera();
      }
      else if (param.get_name() == "exposure_time")
        err = exposure_time(param.as_double());
      else if (param.get_name() == "gain")
        err = gain(param.as_double());
      else if (param.get_name() == "white_balance_blue")
        err = white_balance_blue(param.as_double());
      else if (param.get_name() == "white_balance_green")
        err = white_balance_green(param.as_double());
      else if (param.get_name() == "white_balance_red")
        err = white_balance_red(param.as_double());
      else if (param.get_name() == "rotate")
        err = rotate(param.as_string());

      if (err != IMV_OK)
      {
        result.successful = false;
        result.reason =
            std::string("Failed to set ") + param.get_name() + ": " + get_imv_error_string(err);
      }
    }
    return result;
  }

  void reinit_camera()
  {
    int err;
    // 打开相机
    while (rclcpp::ok())
    {
      RCLCPP_WARN(get_logger(), "尝试重新打开相机");
      err = IMV_DestroyHandle(dev_);
      if (err != IMV_OK)
        RCLCPP_WARN(get_logger(), "摧毁设备句柄失败：%s", get_imv_error_string(err).c_str());
      try
      {
        err = open_camera(camera_id_);
        if (err != IMV_OK) throw std::runtime_error(get_imv_error_string(err));
        RCLCPP_INFO(get_logger(), "重新打开相机成功");
        break;
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR(get_logger(), "打开相机失败：%s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    // 参数设置
    exposure_time(exposure_time_);
    gain(gain_);
    white_balance_blue(white_balance_blue_);
    white_balance_green(white_balance_green_);
    white_balance_red(white_balance_red_);

    // 开始拉流
    while (rclcpp::ok())
    {
      try
      {
        err = start_streaming();
        if (err != IMV_OK) throw std::runtime_error(get_imv_error_string(err));
        RCLCPP_INFO(get_logger(), "拉流成功");
        break;
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR(get_logger(), "拉流失败：%s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
        reinit_camera();
      }
    }
  }

  void camera_thread()
  {
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    camera_info_msg_.header.frame_id = "0";
    sensor_msgs::msg::Image image_msg_;
    image_msg_.encoding = "bgr8";
    image_msg_.is_bigendian = false;
    IMV_Frame frame;
    IMV_PixelConvertParam pc_params;
    pc_params.eBayerDemosaic = demosaicNearestNeighbor;
    pc_params.eDstPixelFormat = gvspPixelBGR8;
    int err;
    while (rclcpp::ok())
    {
      err = IMV_GetFrame(dev_, &frame, 100);
      if (err == IMV_OK)
      {
        image_msg_.header.stamp = camera_info_msg_.header.stamp = this->now();
        size_t data_size = frame.frameInfo.width * frame.frameInfo.height * 3;
        image_msg_.data.resize(data_size);

        pc_params.nWidth = frame.frameInfo.width;
        pc_params.nHeight = frame.frameInfo.height;
        pc_params.ePixelFormat = frame.frameInfo.pixelFormat;
        pc_params.pSrcData = frame.pData;
        pc_params.nSrcDataLen = frame.frameInfo.size;
        pc_params.nPaddingX = frame.frameInfo.paddingX;
        pc_params.nPaddingY = frame.frameInfo.paddingY;
        pc_params.pDstBuf = image_msg_.data.data();
        pc_params.nDstBufSize = data_size;
        err = IMV_PixelConvert(dev_, &pc_params);

        if (rotate_code_ == -1)
        {
          image_msg_.width = camera_info_msg_.width = pc_params.nWidth;
          image_msg_.height = camera_info_msg_.height = pc_params.nHeight;
        }
        else
        {
          cv::Mat tmp(pc_params.nHeight, pc_params.nWidth, CV_8UC3, image_msg_.data.data());
          cv::rotate(tmp, tmp, rotate_code_);
          image_msg_.data.assign((const unsigned char *)tmp.data, tmp.dataend);
          image_msg_.width = camera_info_msg_.width = tmp.cols;
          image_msg_.height = camera_info_msg_.height = tmp.rows;
        }
        image_msg_.step = image_msg_.width * 3;

        if (err == IMV_OK)
          camera_pub_.publish(image_msg_, camera_info_msg_);
        else
          RCLCPP_WARN(get_logger(), "转换像素失败：%s", get_imv_error_string(err).c_str());

        err = IMV_ReleaseFrame(dev_, &frame);
        if (err == IMV_OK)
          continue;  // 直接进行下一轮的取图，避免进入下方重启相机的逻辑
        else
          RCLCPP_ERROR(get_logger(), "释放图像缓存失败：%s", get_imv_error_string(err).c_str());
      }
      else
        RCLCPP_ERROR(get_logger(), "获取图像缓存失败：%s", get_imv_error_string(err).c_str());

      // 重启相机
      reinit_camera();
    }
  }

 public:
  explicit DahuaCameraNode(const rclcpp::NodeOptions &options) : Node("dahua_camera_node", options)
  {
    // 初始化相机
    camera_id_ = this->declare_parameter<int>("camera_id", 0);
    open_camera(camera_id_);
    // 参数设置
    exposure_time(this->declare_parameter<double>("exposure_time", 5000));
    gain(this->declare_parameter<double>("gain", 1.0));
    white_balance_blue(this->declare_parameter<double>("white_balance_blue", 1.0));
    white_balance_green(this->declare_parameter<double>("white_balance_green", 1.0));
    white_balance_red(this->declare_parameter<double>("white_balance_red", 1.0));
    rotate(this->declare_parameter<std::string>("rotate", "NONE"));
    // 开始拉流
    start_streaming();

    bool use_sensor_data_qos = this->declare_parameter<bool>("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "frame", qos);

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DahuaCameraNode::on_set_parameters_callback, this, std::placeholders::_1));

    thread_ = std::thread(&DahuaCameraNode::camera_thread, this);
  }

  ~DahuaCameraNode()
  {
    if (thread_.joinable()) thread_.join();

    IMV_StopGrabbing(dev_);
    IMV_Close(dev_);
  }
};
}  // namespace rm_camera

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(rm_camera::DahuaCameraNode)