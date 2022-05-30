#ifndef __WORK_THREAD_CAPTURETHREAD_HPP__
#define __WORK_THREAD_CAPTURETHREAD_HPP__

#include "BaseThread.hpp"

#include "config/config.hpp"

#include <thread>
#include <functional>

namespace rmcv::work_thread
{
    class CaptureThread : BaseThread
    {
    private:
        std::thread thread_;

        std::string target_;
        std::function<void(void)> target_func_;

        void image(const std::string path);
        void video(const std::string path); // TODO
        void camera(int32_t id, double exposure_time, double gain, double white_balance_red, double white_balance_green, double white_balance_blue);

    public:
        CaptureThread(const rmcv::config::Config &cfg);
        void up();
    };

}

#endif