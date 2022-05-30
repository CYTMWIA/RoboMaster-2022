#include <iostream>
#include <cstdio>
#include <thread>
#include <chrono>
#include <thread>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "logging.hpp"
#include "threading.hpp"

#include "capture/daheng.hpp"
#include "detect/model.hpp"
#include "predict/PnpSolver.hpp"
#include "detect/cv_armor.hpp"
#include "predict/AdaptiveEKF.hpp"
#include "predict/UniformLinearMotionR.hpp"
#include "io/serial.hpp"

// 使用OpenCV窗口调参
#define DEBUG_WITH_OPENCV_WINDOW 0

using namespace rmcv;

void task_capture(const Config& cfg)
{
    auto cp = capture::DahengCapturer(1);
    cp.set_exposure_time(cfg.capture.camera.exposure_time);
    cp.set_gain(cfg.capture.camera.gain);
    cp.set_white_balance(cfg.capture.camera.white_balance_red, cfg.capture.camera.white_balance_green, cfg.capture.camera.white_balance_blue);

    while (true)
    {
        VariableCenter<cv::Mat>::set("src_image", cp.next());
    }
}

void task_detect(const Config& cfg)
{
    auto md = detect::Model("/home/rm/models/model-opt-int8.xml", "/home/rm/models/model-opt-int8.bin");
    while (true)
    {
        VariableCenter<std::vector<detect::BoundingBox>>::set("detections", md(VariableCenter<cv::Mat>::get("src_image")));
    }
    
}

int main(int, char **)
{
#define TEST_LOGGING 0
#if TEST_LOGGING
    __LOG_DEBUG("这是调试信息");
    __LOG_INFO("这是普通信息");
    __LOG_WARNING("这是警告信息");
    __LOG_ERROR("这是错误信息");
#endif

    Config cfg;
    __LOG_INFO("读取配置文件");
    cfg.read("config.toml");

    io::Serial ser{};

    std::unordered_map<ThreadCode, std::thread> threads_pool;
    MessageBus& msgbus = MessageBus::connect();

    msgbus.checkin(ThreadCode::main);

    auto t1 = std::thread(task_capture, std::ref(cfg));
    auto t2 = std::thread(task_detect, std::ref(cfg));

    AdaptiveEKF<6,3> ekf;

    // 预测过程协方差
    ekf.Q(0, 0) = 1;
    ekf.Q(1, 1) = 10;
    ekf.Q(2, 2) = 1;
    ekf.Q(3, 3) = 10;
    ekf.Q(4, 4) = 1;
    ekf.Q(5, 5) = 10;
    // 观测过程协方差
    ekf.R(0, 0) = 20;
    ekf.R(1, 1) = 20;
    ekf.R(2, 2) = 20;
	
    bool ekf_init = false;
    predict::ulmr::Predict predict;
    predict::ulmr::Measure measure;


    auto ps = predict::PnpSolver("/home/rm/RMCV/build/BuBinyTou.xml");
    const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
    auto last_time = std::chrono::steady_clock::now();
    while (true)
    {
        auto img = VariableCenter<cv::Mat>::get("src_image");
        // auto detections = md(img);
        auto detections = VariableCenter< std::vector<detect::BoundingBox> >::get("detections");
        
        if (detections.empty())
        {
            // __LOG_DEBUG("NOT THING");
            cv::imshow("SHOW", img);
            cv::waitKey(40);
            last_time = std::chrono::steady_clock::now();
            ekf_init = false;
            continue;
        }
        // __LOG_DEBUG("SOME THING");

        auto b = detections[0];

        // __LOG_DEBUG("({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f})", b.pts[0].x, b.pts[0].y, b.pts[1].x, b.pts[1].y, b.pts[2].x, b.pts[2].y, b.pts[3].x, b.pts[3].y);
        // b = detect::fix_boundingbox(b, img);
        // __LOG_DEBUG("({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f}), ({:.1f}, {:.1f})", b.pts[0].x, b.pts[0].y, b.pts[1].x, b.pts[1].y, b.pts[2].x, b.pts[2].y, b.pts[3].x, b.pts[3].y);
        cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
        cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
        cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
        cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
        cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);

        auto pose = ps.solve(predict::ArmorType::kSmallArmor, b.pts);

        if (!ekf_init)
        {
            Eigen::Matrix<double, 6, 1> Xr;
            Xr << pose.x,0,pose.y,0,pose.z,0;
            ekf.init(Xr);
            ekf_init = true;
        }
        
        // predict.delta_t = 0;
        predict.delta_t = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-last_time).count();

        ekf.predict(predict);
        Eigen::Matrix<double, 3, 1> Yr;
        Yr(0,0) = pose.x;
        Yr(1,0) = pose.y;
        Yr(2,0) = pose.z;
        ekf.update(measure, Yr);

        // std::cout << ekf.Xe << std::endl;
        // __LOG_DEBUG("RAW {:.1f}, {:.1f}, {:.1f}", );
        __LOG_DEBUG("R {:.1f}, {:.1f}, {:.1f} T {:.1f}, {:.1f}, {:.1f}; S {:.2f}, {:.2f}, {:.2f}", pose.x, pose.y, pose.z, ekf.Xe[0], ekf.Xe[2], ekf.Xe[4], ekf.Xe[1], ekf.Xe[3], ekf.Xe[5]);
        // __LOG_DEBUG("D {:.2f} | P {:.2f} | Y {:.2f} | R {:.2f}", pose.distance(), pose.theta_x, pose.theta_y, pose.theta_z);
        
        ser.vofa_justfloat(pose.x, pose.y, pose.z, ekf.Xe[0], ekf.Xe[2], ekf.Xe[4], ekf.Xe[1], ekf.Xe[3], ekf.Xe[5]);
        cv::imshow("SHOW", img);
        cv::waitKey(40);
        last_time = std::chrono::steady_clock::now();
    }
    // __LOG_INFO("启动捕获线程...");
    // threads_pool[ThreadCode::capture] = std::thread(start_capture, std::ref(cfg.capture));
    
    // __LOG_INFO("捕获线程上线");

    // __LOG_INFO("启动识别线程...");
    // threads_pool[ThreadCode::detect] = std::thread(start_detect, std::ref(cfg.detect));
    
    // __LOG_INFO("识别线程上线");

    // __LOG_INFO("启动策略线程...");
    // threads_pool[ThreadCode::strategy] = std::thread(start_strategy, std::ref(cfg.strategy));

    // __LOG_INFO("策略线程上线");

    // if (cfg.communicate.enable)
    // {
    //     __LOG_INFO("启动通信线程");
    // }

    // if (cfg.record.enable)
    // {
    //     __LOG_INFO("启动录像线程");
    // }

    // if (cfg.web.enable)
    // {
    //     __LOG_INFO("启动Web线程");
    //     threads_pool[ThreadCode::web] = std::thread(start_web, std::ref(cfg.web));
    // }
    

#if DEBUG_WITH_OPENCV_WINDOW
    while (true)
    {
        auto img = VariableCenter<cv::Mat>::get("detect_image");
        auto detections = VariableCenter<std::vector<BoundingBox>>::get("detect_result");

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
        if (VariableCenter<cv::Mat>::exist("debug_fb")) cv::imshow("DEBUG", VariableCenter<cv::Mat>::get("debug_fb", true));
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
