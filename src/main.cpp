#include <thread>

#include <opencv2/opencv.hpp>

#include "config/config.hpp"
#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "detect/boundingbox.hpp"
#include "predict/predict.hpp"
#include "communicate/communicate.hpp"

#include "capture.hpp"
#include "detect.hpp"
#include "communicate.hpp"

using namespace rmcv;
using namespace config;
using namespace threading;
using namespace detect;
using namespace communicate;
using namespace predict;

int main(int, char **)
{
    Vofa vofa("192.168.137.1", 1347);

    std::vector<double> a = {1,2,3,4};
    while (true)
    {
        vofa.justfloat(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    

    Config cfg;
    __LOG_INFO("读取配置文件");
    cfg.read("config.toml");

    /**********************************
     * 启动线程
     */
    std::vector<std::thread> thread_pool;

    thread_pool.push_back(std::thread(thread_capture, std::ref(cfg)));
    thread_pool.push_back(std::thread(thread_detect, std::ref(cfg)));
    if (cfg.communicate.enable)
        thread_pool.push_back(std::thread(thread_communicate, std::ref(cfg)));

    /**********************************
     * EKF 相关
     */

    AdaptiveEKF<6, 3> ekf;
    bool ekf_init = false;
    ulm::Predict predict;
    predict.delta_t = 0;
    ulm::Measure measure;

    // 预测过程协方差
    ekf.Q(0, 0) = 0.1;
    ekf.Q(1, 1) = 100;
    ekf.Q(2, 2) = 0.1;
    ekf.Q(3, 3) = 100;
    ekf.Q(4, 4) = 0.1;
    ekf.Q(5, 5) = 100;
    // 观测过程协方差
    ekf.R(0, 0) = 1;   // pitch
    ekf.R(1, 1) = 1;   // yaw
    ekf.R(2, 2) = 500; // distance

    /**********************************
     * 计算 相关
     */

    auto pnp = PnpSolver(cfg.camera.calibration_file);
    auto aimer = Aimer();

    int history_size = 5;
    std::vector<float> pitch_history;
    std::vector<float> yaw_history;

    auto first_img = RoslikeTopic<cv::Mat>::get("capture_image");
    auto img_width = first_img.cols, img_height = first_img.rows;
    auto img_x_center = img_width / 2.0, img_y_center = img_height / 2.0;
    while (true)
    {
        // 数据更新
        auto detections = RoslikeTopic<std::vector<BoundingBox>>::get("detect_result");
        auto robot_status = RoslikeTopic<RobotStatus>::get("robot_status", true); // 允许旧数据
        // __LOG_DEBUG("RobotStatus Pitch {}", robot_status.pitch);
        // __LOG_DEBUG("RobotStatus BS {}", robot_status.bullet_speed);

        // aimer.bullet_speed(robot_status.bullet_speed * 1000);
        aimer.bullet_speed(16.2 * 1000);

        CmdToEc cmd2ec;

        // 计算/决策
        if (!detections.empty())
        {
            auto target = detections[0];
            __LOG_DEBUG("{:.1f} {:.1f}, {:.1f} {:.1f}, {:.1f} {:.1f}, {:.1f} {:.1f}", target.pts[0].x, target.pts[0].y, target.pts[1].x, target.pts[1].y, target.pts[2].x, target.pts[2].y, target.pts[3].x, target.pts[3].y);

            auto pnp_result = pnp.solve(kSmallArmor, target.pts);
            // __LOG_DEBUG("PnP X {}, Y {}, Z {}", pnp_result.x, pnp_result.y, pnp_result.z);
            __LOG_DEBUG("PnP P {}, Y {}, D {}", pnp_result.pitch, pnp_result.yaw, pnp_result.distance);

            if (!ekf_init)
            {
                // __LOG_DEBUG("EKF Init");
                Eigen::Matrix<double, 6, 1> Xr;
                Xr << pnp_result.x, 0, pnp_result.y, 0, pnp_result.z, 0;
                ekf.init(Xr);
                ekf_init = true;
            }

            ekf.predict(predict);
            Eigen::Matrix<double, 3, 1> Yr;
            Yr << pnp_result.pitch, pnp_result.yaw, pnp_result.distance;
            ekf.update(measure, Yr);
            // __LOG_DEBUG("EKF X {}, Y {}, Z {}", ekf.Xe[0], ekf.Xe[2], ekf.Xe[4]);

            // auto aim_result = aimer.aim_static({ekf.Xe[0], ekf.Xe[2], ekf.Xe[4]}, robot_status.pitch / 180.0 * M_PI);
            auto aim_result = aimer.aim_static({pnp_result.x, pnp_result.y, pnp_result.z}, robot_status.pitch / 180.0 * M_PI);

            cmd2ec.pitch = aim_result.pitch / M_PI * 180.0;
            cmd2ec.yaw = aim_result.yaw / M_PI * 180.0;
        }
        else // 目标丢失
        {
            ekf_init = false;
            cmd2ec.pitch = cmd2ec.yaw = 0;
        }

        RoslikeTopic<CmdToEc>::set("cmd_to_ec", std::move(cmd2ec));

        if (!cfg.debug.enable_window)
            continue;

        // 可视化检测结果
        auto img = RoslikeTopic<cv::Mat>::get("capture_image");
        const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
        for (const auto &b : detections)
        {
            // for (int i=0;i<4;i++) std::cout<< b.pts[i].x << " " << b.pts[i].y << std::endl;
            cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
            cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
            cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
            cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
            cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
        }
        cv::imshow("SHOW", img);
        cv::waitKey(5);
    }

    return 0;
}
