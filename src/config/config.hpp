#ifndef __CONFIG_CONFIG_HPP__
#define __CONFIG_CONFIG_HPP__

#include <string>
#include <vector>

#include <toml.hpp>

namespace rmcv::config
{
    using config_path = std::string;

    class Config
    {
    public:
        // 模块参数

        struct Capture
        {
            std::string target; // 捕获目标 可选：video, image, camera
        } capture;

        struct Communicate
        {
            bool enable;        // 是否启用通信线程
            std::string target; // 通信目标 可选：serial, vofa
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
            int32_t id;                   // 相机id
            double exposure_time;         // 相机曝光时间
            double gain;                  // 相机增益
            double white_balance_red;     // 相机白平衡 - 红色通道
            double white_balance_green;   // 相机白平衡 - 绿色通道
            double white_balance_blue;    // 相机白平衡 - 蓝色通道
            std::string calibration_file; // 相机标定文件
        } camera;

        struct Model // 目标检测模型
        {
            std::string onnx_file; // onnx 模型路径
            std::string bin_file;  // bin 模型路径
            std::string xml_file;  // xml 模型路径
        } model;

        struct Ekf // 扩展卡尔曼
        {
            std::vector<double> q; // 预测过程协方差 对角线
            std::vector<double> r; // 观测过程协方差 对角线
        } ekf;

        struct Serial // 串口
        {
            std::string port;  // 串口接口
            int32_t baud_rate; // 波特率
        } serial;

        struct Vofa // VOFA+
        {
            std::string ip; // IP
            int32_t port;   // 端口
        } vofa;

        // 其他参数

        struct Debug
        {
            bool enable_window; // 是否启动窗口（可视化检测结果）
        } debug;

        // 函数

        void read(const config_path &path);

    private:
        toml::value data_;
        config_path path_;

        template <typename T>
        T dot_find_or(std::string dotkeys, T fallback);

        std::vector<std::string> split_string(const std::string str, const char split_char='.');

        template <typename T>
        void print_kv(std::string key, const T& value);

        template <typename T>
        void print_kv(std::string key, const std::vector<T>& vec);
    };
}

#endif