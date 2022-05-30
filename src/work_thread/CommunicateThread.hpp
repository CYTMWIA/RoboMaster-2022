#ifndef __WORK_THREAD_COMMUNICATETHREAD_HPP__
#define __WORK_THREAD_COMMUNICATETHREAD_HPP__

/*
 * 视觉接收到的信息名为 "robot_status" 类型为 RobotStatus
 * 发送给电控的信息名为 "cmd_to_ec"    类型为 CmdToEc
 * （调试用）发送给 vofa 的信息名为 "vofa_justfloat" 类型为 std::vector<float>
 */

#include "BaseThread.hpp"

#include "config/config.hpp"
#include "communicate/communicate.hpp"

#include <thread>
#include <functional>

namespace rmcv::work_thread
{
    class CommunicateThread : BaseThread
    {
    private:
        std::thread thread_;

        int32_t freq_;
        std::string target_;
        std::function<void(void)> target_func_;

        void cycle(const std::function<void(void)> &func);
        void vofa(const std::string ip, const uint16_t port);
        void serial(const std::string port, const uint32_t baud_rate);

    public:
        CommunicateThread(const rmcv::config::Config &cfg);
        void up();
    };
}

#endif