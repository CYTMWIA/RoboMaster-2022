#include <thread>

#include <opencv2/opencv.hpp>

#include "logging/logging.hpp"

#include "VideoCapture.hpp"

namespace rmcv::capture
{
    VideoCapture::VideoCapture(Path path, bool loop):
        path_(path),
        loop_(loop)
    {
        using namespace std::chrono_literals;

        cap_ = cv::VideoCapture(path_, cv::CAP_FFMPEG);
        
        auto fps = cap_.get(cv::CAP_PROP_FPS);
        frame_interval_ = std::chrono::duration_cast<std::chrono::milliseconds>(1000ms / fps);
    }

    cv::Mat VideoCapture::next()
    {
        std::this_thread::sleep_until(next_frame_time_);
        next_frame_time_ += frame_interval_;

        cv::Mat img;

        auto ok = cap_.read(img);
        if (!ok)
            __LOG_ERROR_AND_EXIT("捕获帧失败");

        if (loop_ && cap_.get(cv::CAP_PROP_POS_FRAMES)==cap_.get(cv::CAP_PROP_FRAME_COUNT)-1)
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);

        return img;
    }
}