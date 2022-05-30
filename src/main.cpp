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
#include "predict/pnp_solver.hpp"

// 使用OpenCV窗口调参
#define DEBUG_WITH_OPENCV_WINDOW 0

using namespace rmcv;

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

    std::unordered_map<ThreadCode, std::thread> threads_pool;
    MessageBus& msgbus = MessageBus::connect();

    msgbus.checkin(ThreadCode::main);

    auto vc = capture::DahengCapturer(1);
    vc.set_exposure_time(cfg.capture.camera.exposure_time);
    vc.set_gain(cfg.capture.camera.gain);
    vc.set_white_balance(cfg.capture.camera.white_balance_red, cfg.capture.camera.white_balance_green, cfg.capture.camera.white_balance_blue);
    
    auto md = detect::Model("/home/rm/models/model-opt-int8.xml", "/home/rm/models/model-opt-int8.bin");
    auto ps = predict::PnpSolver("/home/rm/RMCV/build/BuBinyTou.xml");
    while (true)
    {
        auto img = vc.next();
        auto detections = md(img);

        const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
        for (const auto &b: detections)
        {
            cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
            cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
            cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
            cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
            cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);

            auto pose = ps.solve(predict::ArmorType::kSmallArmor, b.pts);
            __LOG_DEBUG("DIS {:.2f}; R {:.2f}, {:.2f}, {:.2f}", pose.distance(), pose.rx, pose.ry, pose.rz);
        }
        cv::imshow("SHOW", img);
        cv::waitKey(5);
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
