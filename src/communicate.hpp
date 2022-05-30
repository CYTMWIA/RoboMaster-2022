#ifndef __RMCV_COMMUNICATE_HPP__
#define __RMCV_COMMUNICATE_HPP__

#include "config/config.hpp"

/*
 * 视觉接收到的信息名为 "robot_status" 类型为 RobotStatus
 * 发送给电控的信息名为 "cmd_to_ec"    类型为 CmdToEc
 * （调试用）发送给 vofa 的信息名为 "vofa_justfloat" 类型为 std::vector<float>
 */

namespace rmcv
{
    void thread_communicate(const config::Config &cfg);
}

#endif