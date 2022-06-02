#ifndef __RM_COMMUNICATE_CMD_HPP__
#define __RM_COMMUNICATE_CMD_HPP__

// extern "C" 会使命名空间失效
// https://stackoverflow.com/questions/28996944/extern-c-linkage-inside-c-namespace
// namespace rm_communicate

extern "C"
{
#include "cmd.h"
}

namespace rm_communicate
{
using CmdToCv = ::CmdToCv;
using CmdToEc = ::CmdToEc;
using RobotStatus = ::RobotStatus;
using CvStatus = ::CvStatus;
}  // namespace rm_communicate

#endif