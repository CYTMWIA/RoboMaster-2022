#include "work_thread/strategy_thread.hpp"

#include "common/data.hpp"
#include "common/logging.hpp"
#include "common/threading.hpp"
#include "communicate/communicate.hpp"
#include "detect/detect.hpp"
#include "predict/predict.hpp"

namespace rm_work_thread
{
StrategyThread::StrategyThread(const rm_config::Config &cfg)
    : pnp_solver_(cfg.camera.calibration_file)
{
  // 预测过程协方差
  for (int i = 0; i < 6; i++) kf_.Q(i, i) = cfg.kalman_filter.q[i];
  // 观测过程协方差
  for (int i = 0; i < 3; i++) kf_.R(i, i) = cfg.kalman_filter.r[i];

  std::cout << kf_.Q << std::endl;
  std::cout << kf_.R << std::endl;
}

void StrategyThread::run()
{
  using namespace rm_config;
  using namespace rm_threading;
  using namespace rm_detect;
  using namespace rm_communicate;
  using namespace rm_predict;
  using namespace rm_filter;
  using namespace rm_data;

  // 初始化
  auto aimer = Aimer();
  cv::Mat first_img = RoslikeTopic<cv::Mat>::get("capture_image");
  cv::Point2f img_center{(float)(first_img.cols / 2.0), (float)(first_img.rows / 2.0)};
  std::unique_ptr<Armor> plast_target = nullptr;            // 上一次识别目标
  auto lats_found_time = std::chrono::steady_clock::now();  // 上一次找到目标的时间
  // auto fps = rm_util::FpsCounter();
  while (true)
  {
    // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});
    /*
     * 数据更新
     */
    auto detections = RoslikeTopic<std::vector<Armor>>::get("detect_result");
    auto robot_status = RoslikeTopic<RobotStatus>::get("robot_status", true);  // 允许旧数据
    // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", { robot_status.pitch,
    // robot_status.yaw});
    __LOG_DEBUG("Robot {:.2f}, {}, {}", robot_status.bullet_speed, robot_status.pitch,
                robot_status.yaw);
    // 转为 弧度
    robot_status.yaw *= M_PI / 180.0;
    robot_status.pitch *= M_PI / 180.0;
    if (robot_status.bullet_speed <= 15) robot_status.bullet_speed = 13.4;
    else if (robot_status.bullet_speed <=18 ) robot_status.bullet_speed = 16.7;

    aimer.bullet_speed(robot_status.bullet_speed);
    // aimer.bullet_speed(16);

    /*
     * 选择目标
     */
    std::unique_ptr<Armor> ptarget = nullptr;
    bool new_target_flag = false;   // 是否为新目标
    bool using_kf_predict = false;  // 使用卡尔曼预测值代替目标
    // TODO 忽略己方装甲板
    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
                       [&](const auto &d) { return d.color_id != robot_status.enemy_color; }),
        detections.end());
    int idx = -1;
    double cmp = -INFINITY;
    // 上次瞄准的装甲板
    if (plast_target != nullptr)
    {
      auto last_rect =
          cv::boundingRect(std::vector<cv::Point2f>{plast_target->pts, plast_target->pts + 4});
      for (int i = 0; i < (int)detections.size(); i++)
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
      if (std::chrono::steady_clock::now() - lats_found_time < std::chrono::milliseconds(200))
      {
        kf_.predict();
        using_kf_predict = true;
      }
      else
      {
        cmp = INFINITY;

        // 上次瞄准的机器人的离画面中心最近的装甲板
        if (plast_target != nullptr)
        {
          for (int i = 0; i < (int)detections.size(); i++)
          {
            auto &det = detections[i];
            if (det.tag_id == plast_target->tag_id)
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
          for (int i = 0; i < (int)detections.size(); i++)
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
      ptarget = std::make_unique<Armor>(detections[idx]);
      lats_found_time = std::chrono::steady_clock::now();
    }

    /*
     * 计算目标当前坐标
     */
    if (ptarget != nullptr)
    {
      // PNP（相机坐标系）
      auto pos = pnp_solver_.solve(ptarget->type, ptarget->pts);
      // 转换为 机器人坐标系
      // 机器人中心
      pos.y += 130;
      pos.z += 48;
      // Pitch 轴
      double len = sqrt(pos.y * pos.y + pos.z * pos.z);
      double rad = atan2(pos.z, pos.y) + robot_status.pitch;
      pos.y = len * cos(rad);
      pos.z = len * sin(rad);
      // Yaw 轴
      len = sqrt(pos.x * pos.x + pos.y * pos.y);
      rad = atan2(pos.y, pos.x) + robot_status.yaw;
      pos.x = len * cos(rad);
      pos.y = len * sin(rad);

      // __LOG_DEBUG("{:.2f}, {:.2f}, {:.2f}", pos.x, pos.y, pos.z);

      if (new_target_flag)
      {
        // 重置ekf
        __LOG_DEBUG("重置滤波器");
        kf_.init(pos.x, pos.y, pos.z);
      }
      else
      {
        kf_.predict();
        kf_.update(pos.x, pos.y, pos.z);
      }
    }

    CmdToEc cmd2ec = {0, 0, 0};
    if (ptarget != nullptr || using_kf_predict)
    {
      RoslikeTopic<std::vector<float>>::set("vofa_justfloat",
                                            {(float)kf_.X[0], (float)kf_.X[2], (float)kf_.X[4]});
      __LOG_DEBUG("POS {} {} {}",kf_.X[0], kf_.X[2], kf_.X[4]);
      // 预测，注意：俯仰角计算结果为弧度
      AimResult aim;
      double t_ms = 0, target_distance;
      while (true)
      {
        auto ppos = kf_.predict_without_save(0.01 + t_ms / 1000.0); // 前面的常数为发射延迟（单位为秒）
        aim = aimer({ppos[0], ppos[2], ppos[4]});

        if (!aim.ok) break;

        auto diff = std::abs(aim.flying_time - t_ms);
        // __LOG_DEBUG("{}", aim.flying_time);
        if (diff < 0.1 || t_ms > aim.flying_time)
        {
          target_distance = sqrt(ppos[0] * ppos[0] + ppos[2] * ppos[2]);
          break;
        }
        t_ms += diff * 0.5;
      }
      // aim = aimer({kf_.X[0], kf_.X[2], kf_.X[4]}, robot_status.pitch / 180.0 * M_PI); // 无预测
      if (aim.ok)
      {
        // 转为角度
        aim.pitch *= (180.0 / M_PI);
        aim.yaw *= (180.0 / M_PI);
        __LOG_DEBUG("RAW AIM {} {}", aim.pitch, aim.yaw);
        // 补偿
        target_distance /= 1000;
        if (15 <= robot_status.bullet_speed)
        {
          aim.pitch += -3;
          if (target_distance < 2) aim.pitch += 1.0;
          if (target_distance >= 2) aim.pitch += 1;
          if (target_distance >= 3) aim.pitch += 1;
          if (target_distance >= 4) aim.pitch += -0.5;
          // if (target_distance >= 4) aim.pitch += 0.5;
        }
        else if (10 <= robot_status.bullet_speed)
        {
          // if (target_distance <= 2) aim.pitch += (target_distance - 3) * 7.0;
          aim.pitch += -2;
          if (target_distance >= 3) aim.pitch += -1;
          // else aim.pitch += (target_distance - 3) * 0.3;
          // else aim.pitch += (target_distance-3)*1.0;
          // if (target_distance < 3000) aim.pitch += -5.0;
          // else if  (target_distance < 6000) aim.pitch += -10.0;
        }
        else  // if (10 <= robot_status.bullet_speed)
        {
        }
        aim.yaw += -1.5;

        aim.pitch -= robot_status.pitch * (180.0 / M_PI);
        aim.yaw -= robot_status.yaw * (180.0 / M_PI);
        if (aim.yaw <= -180)
          aim.yaw += 360;
        else if (aim.yaw >= 180)
          aim.yaw -= 360;
        __LOG_DEBUG("AIM {} {} {}", target_distance, aim.pitch, aim.yaw);

        cmd2ec.pitch = std::max(-15.0, std::min((double)aim.pitch, 15.0));
        cmd2ec.yaw = std::max(-15.0, std::min((double)aim.yaw, 15.0));
      }
    }

    /*
     * 发送信号
     */
    // RoslikeTopic<std::vector<float>>::set("vofa_justfloat",
    //                                       {(float)cmd2ec.pitch, (float)cmd2ec.yaw});
    RoslikeTopic<CmdToEc>::set("cmd_to_ec", std::move(cmd2ec));

    // 状态保存
    if (ptarget != nullptr)
    {
      plast_target = std::make_unique<Armor>(*ptarget);
    }
  }
}
}  // namespace rm_work_thread