#include "StrategyThread.hpp"

#include "communicate/communicate.hpp"
#include "detect/detect.hpp"
#include "logging/logging.hpp"
#include "predict/predict.hpp"
#include "threading/threading.hpp"
#include "util/util.hpp"

namespace rmcv::work_thread
{
    StrategyThread::StrategyThread(const rmcv::config::Config &cfg) : pnp_solver_(cfg.camera.calibration_file)
    {
        // 预测过程协方差
        ekf_.set_q_diag(cfg.ekf.q);
        // 观测过程协方差
        ekf_.set_r_diag(cfg.ekf.r);
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

        bool ekf_init = false;
        ulm::Predict predict;
        predict.delta_t = 0;
        ulm::Measure measure;

        auto aimer = Aimer();
        MovingAverageFilter ma_pitch(5);
        MovingAverageFilter ma_yaw(5);
        while (true)
        {
            // 数据更新
            auto detections = RoslikeTopic<std::vector<BoundingBox>>::get("detect_result");
            auto robot_status = RoslikeTopic<RobotStatus>::get("robot_status", true); // 允许旧数据

            // aimer.bullet_speed(robot_status.bullet_speed * 1000);
            aimer.bullet_speed(16.2 * 1000);

            CmdToEc cmd2ec;

            // 计算/决策
            if (!detections.empty())
            {
                auto target = detections[0];
                // __LOG_DEBUG("{:.1f} {:.1f}, {:.1f} {:.1f}, {:.1f} {:.1f}, {:.1f} {:.1f}", target.pts[0].x, target.pts[0].y, target.pts[1].x, target.pts[1].y, target.pts[2].x, target.pts[2].y, target.pts[3].x, target.pts[3].y);

                auto pnp_result = pnp_solver_.solve(kSmallArmor, target.pts);
                // RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {(float)pnp_result.pitch, (float)pnp_result.yaw, (float)pnp_result.distance});

                if (!ekf_init)
                {
                    __LOG_DEBUG("EKF Init");
                    Eigen::Matrix<double, 6, 1> Xr;
                    Xr << pnp_result.x, 0, pnp_result.y, 0, pnp_result.z, 0;
                    ekf_.init(Xr);
                    ekf_init = true;
                }

                ekf_.predict(predict);
                Eigen::Matrix<double, 3, 1> Yr;
                Yr << pnp_result.pitch, pnp_result.yaw, pnp_result.distance;
                ekf_.update(measure, Yr);
                auto xd = sqrt(ekf_.Xe[0] * ekf_.Xe[0] + ekf_.Xe[2] * ekf_.Xe[2] + ekf_.Xe[4] * ekf_.Xe[4]);

                // auto aim_result = aimer.aim_static({ekf_.Xe[0], ekf_.Xe[2], ekf_.Xe[4]}, robot_status.pitch / 180.0 * M_PI);
                auto aim_result = aimer.aim_static({pnp_result.x, pnp_result.y, pnp_result.z}, robot_status.pitch / 180.0 * M_PI);

                cmd2ec.pitch = aim_result.pitch / M_PI * 180.0;
                cmd2ec.yaw = aim_result.yaw / M_PI * 180.0;

                // if (cmd2ec.pitch>40)
                // {
                //     __LOG_DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                // }
                // __LOG_DEBUG(
                //     "({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f})",
                //     target.pts[0].x, target.pts[0].y,
                //     target.pts[1].x, target.pts[1].y,
                //     target.pts[2].x, target.pts[2].y,
                //     target.pts[3].x, target.pts[3].y
                // );
                
            }
            else // 目标丢失
            {
                ekf_init = false;
                cmd2ec.pitch = cmd2ec.yaw = 0;
            }

            // cmd2ec.pitch = limit(cmd2ec.pitch, -0.5, 0.5);
            // cmd2ec.yaw = limit(cmd2ec.yaw, -0.5, 0.5);

            // cmd2ec.pitch = ma_pitch.update(cmd2ec.pitch);
            // cmd2ec.yaw = ma_yaw.update(cmd2ec.yaw);

            RoslikeTopic<std::vector<float>>::set("vofa_justfloat", {cmd2ec.pitch, cmd2ec.yaw, ((float)detections.size())*10});
            RoslikeTopic<CmdToEc>::set("cmd_to_ec", std::move(cmd2ec));
        }
    }
}