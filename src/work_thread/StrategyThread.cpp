#include "StrategyThread.hpp"

#include "communicate/communicate.hpp"
#include "detect/detect.hpp"
#include "logging/logging.hpp"
#include "predict/predict.hpp"
#include "threading/threading.hpp"
#include "util/util.hpp"
#include "filter/filter.hpp"

namespace rmcv::work_thread
{
    StrategyThread::StrategyThread(const rmcv::config::Config &cfg) : pnp_solver_(cfg.camera.calibration_file)
    {
        // 预测过程协方差
        for (int i=0;i<6;i++)
            kf_.Q(i, i) = cfg.kalman_filter.q[i];
        // 观测过程协方差
        for (int i=0;i<3;i++)
            kf_.R(i, i) = cfg.kalman_filter.r[i];

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

        // 初始化

        auto aimer = Aimer();
        cv::Mat first_img = RoslikeTopic<cv::Mat>::get("capture_image");
        cv::Point2f img_center{(float)(first_img.cols/2.0), (float)(first_img.rows/2.0)};

        std::unique_ptr<BoundingBox> plast_target = nullptr; // 上一次识别目标
        while (true)
        {
            // 数据更新
            auto detections = RoslikeTopic<std::vector<BoundingBox>>::get("detect_result");
            auto robot_status = RoslikeTopic<RobotStatus>::get("robot_status", true); // 允许旧数据

            // aimer.bullet_speed(robot_status.bullet_speed * 1000);
            aimer.bullet_speed(16.2 * 1000);

            // 选择目标
            std::unique_ptr<BoundingBox> ptarget = nullptr;
            if (!detections.empty())
            {
                // 步兵策略：选择离画面中心最近的装甲板
                int idx = 0; double dis = INFINITY;
                for (int i=0;i<detections.size();i++)
                {
                    auto &det = detections[i];
                    auto det_enter = (det.pts[0]+det.pts[1]+det.pts[2]+det.pts[3])/4.0;
                    double d = cv::norm(det_enter - img_center);
                    if (d < dis)
                    {
                        idx = i;
                        dis = d;
                    }
                }
                ptarget = std::make_unique<BoundingBox>(detections[idx]);
            }

            // 计算
            CmdToEc cmd2ec = { 0, 0 };
            if (ptarget!=nullptr)
            {
                // 新目标判别
                bool new_target_flag = false;
                if (plast_target == nullptr) new_target_flag = true;
                else
                {
                    auto target_rect = cv::boundingRect(std::vector<cv::Point2f>(&ptarget->pts[0], &ptarget->pts[4]));
                    auto last_target_rect = cv::boundingRect(std::vector<cv::Point2f>(&plast_target->pts[0], &plast_target->pts[4]));
                    new_target_flag = (target_rect & last_target_rect).area() <= 0;
                }
                // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {ptarget->pts[0].x, ptarget->pts[0].y, ptarget->pts[1].x, ptarget->pts[1].y, ptarget->pts[2].x, ptarget->pts[2].y, ptarget->pts[3].x, ptarget->pts[3].y});

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

                // 俯仰角计算
                auto aim = aimer.aim_static({kf_.X[0], kf_.X[2], kf_.X[4]}, robot_status.pitch / 180.0 * M_PI);

                // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {(float)pos.pitch, (float)pos.yaw, (float)pos.distance});
                RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {(float)pos.x, (float)pos.y, (float)pos.z, (float)kf_.X[0], (float)kf_.X[1], (float)kf_.X[2], (float)kf_.X[3], (float)kf_.X[4], (float)kf_.X[5]});

                // 转为角度
                cmd2ec.pitch = aim.pitch*(180.0/M_PI);
                cmd2ec.yaw = aim.yaw*(180.0/M_PI);
            }

            // 发送信号
            // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {(float)cmd2ec.pitch, (float)cmd2ec.yaw});
            RoslikeTopic<CmdToEc>::set("cmd_to_ec", std::move(cmd2ec));

            // 状态保存
            if (ptarget!=nullptr)
                plast_target = std::make_unique<BoundingBox>(*ptarget);
            else
                plast_target = nullptr;
        }
    }
}