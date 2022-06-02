#ifndef __RM_NODE_COMMUNICATE_THREAD_HPP__
#define __RM_NODE_COMMUNICATE_THREAD_HPP__

/*
 * 视觉接收到的信息名为 "robot_status" 类型为 RobotStatus
 * 发送给电控的信息名为 "cmd_to_ec"    类型为 CmdToEc
 * （调试用）发送给 vofa 的信息名为 "vofa_justfloat" 类型为 std::vector<float>
 */

#include <functional>
#include <thread>

#include "base_thread.hpp"
#include "rm_communicate/communicate.hpp"
#include "rm_config/config.hpp"

namespace rm_node
{
DECL_WORKTHTREAD(CommunicateThread)

private:
int32_t freq_;

bool vofa_enable_;
std::string vofa_ip_;
uint16_t vofa_port_;

bool serial_enable_;
std::string serial_port_;
uint32_t serial_baud_rate_;

void cycle(const std::function<void(void)> &func);

public:
CommunicateThread(const rm_config::Config &cfg);

DECL_WORKTHTREAD_END()
}  // namespace rm_node

#endif