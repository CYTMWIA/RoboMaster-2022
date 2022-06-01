#include "strategy_thread.hpp"

#include "communicate/communicate.hpp"
#include "detect/detect.hpp"
#include "filter/filter.hpp"
#include "logging/logging.hpp"
#include "predict/predict.hpp"
#include "rm_data.hpp"
#include "threading/threading.hpp"
#include "util/util.hpp"

namespace rmcv::work_thread
{
StrategyThread::StrategyThread(const rmcv::config::Config &cfg)
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
  using namespace rmcv;
  using namespace config;
  using namespace threading;
  using namespace detect;
  using namespace communicate;
  using namespace predict;
  using namespace util;
  using namespace filter;
  using namespace rm_data;

  // 初始化
  auto aimer = Aimer();
  cv::Mat first_img = RoslikeTopic<cv::Mat>::get("capture_image");
  cv::Point2f img_center{(float)(first_img.cols / 2.0), (float)(first_img.rows / 2.0)};
  std::unique_ptr<BoundingBox> plast_target = nullptr;      // 上一次识别目标
  auto lats_found_time = std::chrono::steady_clock::now();  // 上一次找到目标的时间
  auto fps = rmcv::util::FpsCounter();
  while (true)
  {
    // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {fps.tick()});
    /*
     * 数据更新
     */
    auto detections = RoslikeTopic<std::vector<BoundingBox>>::get("detect_result");
    auto robot_status = RoslikeTopic<RobotStatus>::get("robot_status", true);  // 允许旧数据

    // aimer.bullet_speed(robot_status.bullet_speed * 1000);
    aimer.bullet_speed(16);

    /*
     * 选择目标
     */
    std::unique_ptr<BoundingBox> ptarget = nullptr;
    bool new_target_flag = false;   // 是否为新目标
    bool using_kf_predict = false;  // 使用卡尔曼预测值代替目标
    // TODO 忽略己方装甲板
    int idx = -1;
    double cmp = -INFINITY;
    // 上次瞄准的装甲板
    if (plast_target != nullptr)
    {
      auto last_rect =
          cv::boundingRect(std::vector<cv::Point2f>{plast_target->pts, plast_target->pts + 4});
      for (int i = 0; i < detections.size(); i++)
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
      if (std::chrono::steady_clock::now() - lats_found_time < std::chrono::milliseconds(250))
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
          for (int i = 0; i < detections.size(); i++)
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
          for (int i = 0; i < detections.size(); i++)
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
      lats_found_time = std::chrono::steady_clock::now();
    }

    /*
     * 计算目标当前坐标
     */
    if (ptarget != nullptr)
    {
      // PNP
      auto pos = pnp_solver_.solve(ARMOR_SMALL, ptarget->pts);
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

    CmdToEc cmd2ec = {0, 0};
    if (ptarget != nullptr || using_kf_predict)
    {
      // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {(float)kf_.X[0], (float)kf_.X[2],
      // (float)kf_.X[4]});

      // 预测，注意：俯仰角计算结果为弧度
      AimResult aim, last_aim;
      double t_ms = 0, last_diff = INFINITY;
      while (true)
      {
        auto ppos = kf_.predict_without_save(t_ms / 1000.0);
        aim = aimer({ppos[0], ppos[2], ppos[4]}, robot_status.pitch / 180.0 * M_PI);

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
    RoslikeTopic<std::vector<float>>::set("vofa_justfloat",
                                          {(float)cmd2ec.pitch, (float)cmd2ec.yaw});
    RoslikeTopic<CmdToEc>::set("cmd_to_ec", std::move(cmd2ec));

    // 状态保存
    if (ptarget != nullptr)
    {
      plast_target = std::make_unique<BoundingBox>(*ptarget);
    }
  }
}
}  // namespace rmcv::work_thread