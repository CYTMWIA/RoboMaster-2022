#ifndef __WORK_THREAD_DETECTTHREAD_HPP__
#define __WORK_THREAD_DETECTTHREAD_HPP__

#include "BaseThread.hpp"

#include "config/config.hpp"
#include "detect/detect.hpp"

#include <thread>
#include <memory>

namespace rmcv::work_thread
{
    class DetectThread : BaseThread
    {
    private:
        std::thread thread_;

        std::unique_ptr<rmcv::detect::Model> pmodel;

        void run();
    public:
        DetectThread(const rmcv::config::Config &cfg);
        void up();
    };
}

#endif