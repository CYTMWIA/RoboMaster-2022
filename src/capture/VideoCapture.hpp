#ifndef __CAPTURE_VIDEO_HPP__
#define __CAPTURE_VIDEO_HPP__

#include <string>
#include <chrono>

#include "BaseCapture.hpp"

namespace rmcv::capture
{
    class VideoCapture : public BaseCapture
    {
    private:
        Path path_;
        bool loop_;
        cv::VideoCapture cap_;
        std::chrono::milliseconds frame_interval_;
        std::chrono::time_point<std::chrono::steady_clock> next_frame_time_ = std::chrono::steady_clock::now();

    public:
        VideoCapture() = delete;

        /**
         * @brief VideoCapture 构造函数
         *
         * @param path 视频路径
         * @param loop 是否循环播放
         */
        VideoCapture(Path path, bool loop = true);

        cv::Mat next() override;
    };
}

#endif