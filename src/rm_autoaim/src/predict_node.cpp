#include "rm_autoaim/predict_node.hpp"

#include "detect/bounding_box.hpp"
#include "predict/aimer.hpp"
#include "predict/pnp_solver.hpp"
#include "predict/ulm_xyz_kf.hpp"
#include "rm_common/data.hpp"
#include "rm_common/logging.hpp"

namespace rm_autoaim
{
class PredictNode::Impl
{
 public:
  UlmXyzKf kf;
  Aimer aimer;
  PnpSolver pnp_solver;
  std::chrono::time_point<std::chrono::steady_clock> lats_found_time;
  std::shared_ptr<BoundingBox> plast_target;
  Impl(const PredictNodeSettings &settings) : pnp_solver{settings.camera_calibration_file} {}
};

PredictNode::PredictNode(const PredictNodeSettings &settings, const std::string &node_name)
    : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
      pimpl(std::make_unique<Impl>(settings))
{
  // Parameter
  
  // Topic
  sub_ = this->create_subscription<rm_interfaces::msg::BoundingBoxesWithImageSize>(
      settings.input, rclcpp::SensorDataQoS(),
      std::bind(&PredictNode::predict, this, std::placeholders::_1));
  pub_debug_target_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "target_point", rclcpp::SensorDataQoS());
  pub_ =
      this->create_publisher<rm_interfaces::msg::Angle3>(settings.output, rclcpp::SensorDataQoS());
}
PredictNode::~PredictNode() = default;

void PredictNode::predict(rm_interfaces::msg::BoundingBoxesWithImageSize::ConstSharedPtr pbboxes)
{
  using namespace rm_data;
  std::vector<BoundingBox> detections{pbboxes->boundingboxes.size()};
  for (size_t i = 0; i < pbboxes->boundingboxes.size(); i++)
  {
    detections[i].color_id = pbboxes->boundingboxes[i].color_id;
    detections[i].confidence = pbboxes->boundingboxes[i].confidence;
    detections[i].tag_id = pbboxes->boundingboxes[i].tag_id;
    for (int j = 0; j < 4; j++)
    {
      detections[i].pts[j].x = pbboxes->boundingboxes[i].points[j].x;
      detections[i].pts[j].y = pbboxes->boundingboxes[i].points[j].y;
    }
    cv::Point2f img_center{(float)(pbboxes->image_width / 2.0),
                           (float)(pbboxes->image_height / 2.0)};

    // Start Here
    std::shared_ptr<BoundingBox> ptarget = nullptr;
    bool new_target_flag = false;   // 是否为新目标
    bool using_kf_predict = false;  // 使用卡尔曼预测值代替目标
    // TODO 忽略己方装甲板
    int idx = -1;
    double cmp = -INFINITY;
    // 上次瞄准的装甲板
    if (pimpl->plast_target != nullptr)
    {
      auto last_rect = cv::boundingRect(
          std::vector<cv::Point2f>{pimpl->plast_target->pts, pimpl->plast_target->pts + 4});
      for (size_t i = 0; i < detections.size(); i++)
      {
        auto &det = detections[i];
        auto det_rect = cv::boundingRect(std::vector<cv::Point2f>{det.pts, det.pts + 4});
        auto area = (det_rect & last_rect).area();
        if (area > cmp)
        {
          idx = i;
          cmp = area;
        }
      }
    }

    if (idx < 0)
    {
      if (std::chrono::steady_clock::now() - pimpl->lats_found_time <
          std::chrono::milliseconds(250))
      {
        pimpl->kf.predict();
        using_kf_predict = true;
      }
      else
      {
        cmp = INFINITY;

        // 上次瞄准的机器人的离画面中心最近的装甲板
        if (pimpl->plast_target != nullptr)
        {
          for (size_t i = 0; i < detections.size(); i++)
          {
            auto &det = detections[i];
            if (det.tag_id == pimpl->plast_target->tag_id)
            {
              auto det_center = (det.pts[0] + det.pts[1] + det.pts[2] + det.pts[3]) / 4.0;
              double d = cv::norm(det_center - img_center);
              if (d < cmp)
              {
                idx = i;
                cmp = d;
              }
            }
          }
        }

        // 离画面中心最近的装甲板
        if (idx < 0)
        {
          for (size_t i = 0; i < detections.size(); i++)
          {
            auto &det = detections[i];
            auto det_center = (det.pts[0] + det.pts[1] + det.pts[2] + det.pts[3]) / 4.0;
            double d = cv::norm(det_center - img_center);
            if (d < cmp)
            {
              idx = i;
              cmp = d;
            }
          }
        }

        if (idx >= 0)
        {
          new_target_flag = 1;
        }
      }
    }

    if (idx >= 0)
    {
      ptarget = std::make_unique<BoundingBox>(detections[idx]);
      pimpl->lats_found_time = std::chrono::steady_clock::now();
    }

    /*
     * 计算目标当前坐标
     */
    if (ptarget != nullptr)
    {
      // PNP
      auto pos = pimpl->pnp_solver.solve(ARMOR_SMALL, ptarget->pts);
      // __LOG_DEBUG("{:.2f}, {:.2f}, {:.2f}", pos.x, pos.y, pos.z);

      if (new_target_flag)
      {
        // 重置ekf
        __LOG_DEBUG("重置滤波器");
        pimpl->kf.init(pos.x, pos.y, pos.z);
      }
      else
      {
        pimpl->kf.predict();
        pimpl->kf.update(pos.x, pos.y, pos.z);
      }
    }

    geometry_msgs::msg::PointStamped ps;
    ps.header.stamp = this->now();
    ps.point.x = pimpl->kf.X[0];
    ps.point.x = pimpl->kf.X[2];
    ps.point.x = pimpl->kf.X[4];
    pub_debug_target_point_->publish(ps);

    rm_interfaces::msg::Angle3 cmd2ec;
    if (ptarget != nullptr || using_kf_predict)
    {
      // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {(float)kf_.X[0], (float)kf_.X[2],
      // (float)kf_.X[4]});

      // 预测，注意：俯仰角计算结果为弧度
      AimResult aim;
      double t_ms = 0;
      while (true)
      {
        auto ppos = pimpl->kf.predict_without_save(t_ms / 1000.0);
        // TODO 实际 pitch
        aim = pimpl->aimer({ppos[0], ppos[2], ppos[4]}, 0 / 180.0 * M_PI);

        if (!aim.ok) break;

        auto diff = std::abs(aim.flying_time - t_ms);
        // __LOG_DEBUG("{}", aim.flying_time);
        if (diff < 0.1 || t_ms > aim.flying_time) break;
        t_ms += diff * 0.5;
      }
      // aim = aimer({kf_.X[0], kf_.X[2], kf_.X[4]}, robot_status.pitch / 180.0 * M_PI); // 无预测
      if (aim.ok)
      {
        // 转为角度
        cmd2ec.pitch = std::max(-5.0, std::min(aim.pitch * (180.0 / M_PI) * 0.3, 5.0));
        cmd2ec.yaw = std::max(-5.0, std::min(aim.yaw * (180.0 / M_PI) * 0.3, 5.0));
        // cmd2ec.pitch = aim.pitch*(180.0/M_PI);
        // cmd2ec.yaw = aim.yaw*(180.0/M_PI);
      }
    }

    /*
     * 发送信号
     */
    pub_->publish(cmd2ec);

    // 状态保存
    if (ptarget != nullptr)
    {
      pimpl->plast_target = std::make_unique<BoundingBox>(*ptarget);
    }
  }
}

}  // namespace rm_autoaim
