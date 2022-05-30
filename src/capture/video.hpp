#ifndef __CAPTURE_VIDEO_HPP__
#define __CAPTURE_VIDEO_HPP__

#include <string>
#include <chrono>

#include "base.hpp"

namespace rmcv::capture
{
    class VideoCapturer: public BaseCapturer
    {
    private:
        Path path_;
        bool loop_;
        cv::VideoCapture cap_;
        std::chrono::milliseconds frame_interval_;
        std::chrono::time_point<std::chrono::steady_clock> next_frame_time_=std::chrono::steady_clock::now();
    public:
        VideoCapturer() = delete;

        /**
         * @brief VideoCapturer 构造函数
         * 
         * @param path 视频路径
         * @param loop 是否循环播放
         */
        VideoCapturer(Path path, bool loop=true);

        cv::Mat next();
    };
}

#endif