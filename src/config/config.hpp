#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include <string>

#include <toml.hpp>

namespace rmcv
{
    using config_path = std::string;

    class Config
    {
    public:
        struct Record
        {
            bool        enable;     // 是否启动录像
            std::string folder;     // 录像储存目录
        } record;
        
        struct Capture
        {
            std::string target;         // 捕获目标 可选：video, image, camera
            struct Video
            {
                std::string path;       // 视频路径
            } video;
            struct Image
            {
                std::string path;       // 图片路径
            } image;
            struct Camera
            {
                int32_t id;             // 相机id
                double exposure_time;   // 相机增益
                double gain;            // 相机曝光时间
                double white_balance_red;   // 相机白平衡 - 红色通道
                double white_balance_green; // 相机白平衡 - 绿色通道
                double white_balance_blue;  // 相机白平衡 - 蓝色通道
            } camera;
        } capture;

        struct Detect
        {
            std::string model_path;     // 模型路径
        } detect;
        
        struct Strategy
        {
            std::string robot;                      // 机器人代号
            std::string camera_calibration_file;    // 相机标定文件
        } strategy;

        struct Communicate
        {
            bool            enable;     // 是否启用通信线程
            std::string     port;       // 串口接口
            int32_t         baud_rate;  // 波特率
        } communicate;
        
        struct Web
        {
            bool            enable;     // 是否启用Web线程
            std::string     ip;         // IP
            uint16_t        port;       // 端口
        } web;
        
        
        void read(const config_path& path);
    
    private:
        toml::value data_;
        config_path path_;

        template<typename T>
        T dot_find_or(std::string dotkeys, T fallback);
    };
}

#endif