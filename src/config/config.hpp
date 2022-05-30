#ifndef __CONFIG_CONFIG_HPP__
#define __CONFIG_CONFIG_HPP__

#include <string>
#include <vector>
#include <memory>

namespace rmcv::config
{
    class Config
    {
    private:
        // Pimpl: https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Ri-pimpl
        class Impl;
        std::unique_ptr<Impl> pimpl;

    public:
        // 模块参数

        struct Capture
        {
            std::string target; // 捕获目标 可选：video, image, camera
        } capture;

        struct Communicate
        {
            bool enable; // 是否启用通信线程
            // std::string target; // 通信目标 可选：serial, vofa
        } communicate;

        // 对象参数

        struct Video // 视频，当 capture.target 为 video 时生效
        {
            std::string path; // 视频路径
        } video;

        struct Image // 图像，当 capture.target 为 image 时生效
        {
            std::string path; // 图片路径
        } image;

        struct Camera // 相机，当 capture.target 为 camera 时生效
        {
            int32_t id;                   // 相机id（弃用）
            std::string manufacturer;     // 制造商 可选：dahua、daheng
            double exposure_time;         // 相机曝光时间
            double gain;                  // 相机增益
            double white_balance_red;     // 相机白平衡 - 红色通道
            double white_balance_green;   // 相机白平衡 - 绿色通道
            double white_balance_blue;    // 相机白平衡 - 蓝色通道
            std::string calibration_file; // 相机标定文件
        } camera;

        struct Model // 目标检测模型
        {
            std::string path; // 模型路径
        } model;

        struct Ekf // 扩展卡尔曼
        {
            std::vector<double> q; // 预测过程协方差 对角线
            std::vector<double> r; // 观测过程协方差 对角线
        } ekf;

        struct Serial // 串口
        {
            bool enable;       // 是否启用
            std::string port;  // 串口接口
            int32_t baud_rate; // 波特率
        } serial;

        struct Vofa // VOFA+
        {
            bool enable;    // 是否启用
            std::string ip; // IP
            int32_t port;   // 端口
        } vofa;

        // 其他参数

        struct Debug
        {
            bool enable_window; // 是否启动窗口（可视化检测结果）
        } debug;

        // 函数
        Config();
        ~Config();
        void read(const std::string &path);
    };
}

#endif