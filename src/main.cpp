#include <thread>

#include <opencv2/opencv.hpp>

#include "config/config.hpp"
#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "detect/boundingbox.hpp"

#include "capture.hpp"
#include "detect.hpp"

// 使用OpenCV窗口调参
#define DEBUG_WITH_OPENCV_WINDOW 1

using namespace rmcv;
using namespace config;
using namespace threading;
using namespace detect;


int main(int, char **)
{
    Config cfg;
    __LOG_INFO("读取配置文件");
    cfg.read("config.toml");

    auto t1 = std::thread(thread_capture, std::ref(cfg));
    auto t2 = std::thread(thread_detect, std::ref(cfg));


    // io::Serial ser{"/dev/ttyUSB0"};

    // auto t1 = std::thread(task_capture, std::ref(cfg));
    // auto t2 = std::thread(task_detect, std::ref(cfg));

    // AdaptiveEKF<6,3> ekf;

    // // 预测过程协方差
    // ekf.Q(0, 0) = 0.1;
    // ekf.Q(1, 1) = 100;
    // ekf.Q(2, 2) = 0.1;
    // ekf.Q(3, 3) = 100;
    // ekf.Q(4, 4) = 0.1;
    // ekf.Q(5, 5) = 100;
    // // 观测过程协方差
    // ekf.R(0, 0) = 1;
    // ekf.R(1, 1) = 1;
    // ekf.R(2, 2) = 500;
	
    // bool ekf_init = false;
    // predict::ulm::Predict predict;
    // predict::ulm::Measure measure;

    // predict::Aimer aimer;
    // auto ps = predict::PnpSolver("/home/rm/RMCV/build/BuBinyTou.xml");
    // const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
    // auto last_time = std::chrono::steady_clock::now();
    // io::RobotStatus robot_status;
    // robot_status.bullet_speed=15;
    // robot_status.pitch = 0;
    // while (true)
    // {
    //     auto img = threading::RoslikeTopic<cv::Mat>::get("src_image");
    //     // auto detections = md(img);
    //     auto detections = threading::RoslikeTopic< std::vector<detect::BoundingBox> >::get("detections");
        
    //     if (detections.empty())
    //     {
    //         // __LOG_DEBUG("NOT THING");
    //         // cv::imshow("SHOW", img);
    //         // cv::waitKey(40);
    //         last_time = std::chrono::steady_clock::now();
    //         ekf_init = false;

    //         io::CvStatus cs{0, 0};
    //         ser.send(cs);
    //         continue;
    //     }
    //     // __LOG_DEBUG("SOME THING");

    //     auto b = detections[0];

    //     // __LOG_DEBUG("({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f})", b.pts[0].x, b.pts[0].y, b.pts[1].x, b.pts[1].y, b.pts[2].x, b.pts[2].y, b.pts[3].x, b.pts[3].y);
    //     // b = detect::fix_boundingbox(b, img);
    //     // __LOG_DEBUG("({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f})", b.pts[0].x, b.pts[0].y, b.pts[1].x, b.pts[1].y, b.pts[2].x, b.pts[2].y, b.pts[3].x, b.pts[3].y);
    //     cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
    //     cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
    //     cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
    //     cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
    //     cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);

    //     auto pose = ps.solve(predict::ArmorType::kSmallArmor, b.pts);

    //     if (!ekf_init)
    //     {
    //         Eigen::Matrix<double, 6, 1> Xr;
    //         Xr << pose.x, 0, pose.y, 0, pose.z, 0;
    //         ekf.init(Xr);
    //         ekf_init = true;
    //     }
        
    //     // predict.delta_t = 0;
    //     predict.delta_t = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-last_time).count();

    //     ekf.predict(predict);
    //     Eigen::Matrix<double, 3, 1> Yr;
    //     Yr<< pose.pitch, pose.yaw, pose.distance;
    //     ekf.update(measure, Yr);

    //     // std::cout << ekf.Xe << std::endl;
    //     // __LOG_DEBUG("RAW {:.1f}, {:.1f}, {:.1f}", );
    //     // __LOG_DEBUG("R {:.1f}, {:.1f}, {:.1f} T {:.1f}, {:.1f}, {:.1f}; S {:.2f}, {:.2f}, {:.2f}", pose.x, pose.y, pose.z, ekf.Xe[0], ekf.Xe[2], ekf.Xe[4], ekf.Xe[1], ekf.Xe[3], ekf.Xe[5]);
    //     // __LOG_DEBUG("D {:.2f} | P {:.2f} | Y {:.2f} | R {:.2f}", pose.distance(), pose.theta_x, pose.theta_y, pose.theta_z);
    //     ser.update(robot_status);
    //     aimer.bullet_speed(robot_status.bullet_speed*1000);
    //     auto aimd = aimer.aim_static({pose.x, pose.y, pose.z}, robot_status.pitch/180.0*M_PI);
    //     __LOG_DEBUG("dt {:.2f} AIM {}, {}", predict.delta_t, aimd.yaw*(180/M_PI), aimd.pitch*(180/M_PI));
    //     io::CvStatus cs{aimd.pitch*(180/M_PI)/2, aimd.yaw*(180/M_PI)/2};
    //     // if (abs(cs.yaw)<1) cs.yaw=0;
    //     // if(abs(cs.pitch)<2)cs.pitch=0;
    //     ser.send(cs);
    //     // ser.vofa_justfloat(pose.x, pose.y, pose.z, ekf.Xe[0], ekf.Xe[2], ekf.Xe[4], ekf.Xe[1], ekf.Xe[3], ekf.Xe[5]);
    //     // cv::imshow("SHOW", img);
    //     // cv::waitKey(40);

    //     last_time = std::chrono::steady_clock::now();
    // }
    // // __LOG_INFO("启动捕获线程...");
    // // threads_pool[ThreadCode::capture] = std::thread(start_capture, std::ref(cfg.capture));
    
    // // __LOG_INFO("捕获线程上线");

    // // __LOG_INFO("启动识别线程...");
    // // threads_pool[ThreadCode::detect] = std::thread(start_detect, std::ref(cfg.detect));
    
    // // __LOG_INFO("识别线程上线");

    // // __LOG_INFO("启动策略线程...");
    // // threads_pool[ThreadCode::strategy] = std::thread(start_strategy, std::ref(cfg.strategy));

    // // __LOG_INFO("策略线程上线");

    // // if (cfg.communicate.enable)
    // // {
    // //     __LOG_INFO("启动通信线程");
    // // }

    // // if (cfg.record.enable)
    // // {
    // //     __LOG_INFO("启动录像线程");
    // // }

    // // if (cfg.web.enable)
    // // {
    // //     __LOG_INFO("启动Web线程");
    // //     threads_pool[ThreadCode::web] = std::thread(start_web, std::ref(cfg.web));
    // // }
    

#if DEBUG_WITH_OPENCV_WINDOW
    while (true)
    {
        auto img = RoslikeTopic<cv::Mat>::get("capture/image");
        auto detections = RoslikeTopic<std::vector<BoundingBox>>::get("detect/result");

        const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
        for (const auto &b: detections)
        {
            // for (int i=0;i<4;i++) std::cout<< b.pts[i].x << " " << b.pts[i].y << std::endl;
            cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
            cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
            cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
            cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
            cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
        }
        cv::imshow("SHOW", img);
        // if (RoslikeTopic<cv::Mat>::exist("debug_fb")) cv::imshow("DEBUG", RoslikeTopic<cv::Mat>::get("debug_fb", true));
        cv::waitKey(5);
    }
#else
    while (true)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1s);
    }
#endif

    return 0;
}
