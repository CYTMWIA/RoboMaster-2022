#ifndef __COMMUNICATE_CMD_HPP__
#define __COMMUNICATE_CMD_HPP__

// extern "C" 会使命名空间失效
// https://stackoverflow.com/questions/28996944/extern-c-linkage-inside-c-namespace
// namespace rmcv::communicate

extern "C"
{
#include "cmd.h"
}

namespace rmcv::communicate
{
using CmdToCv = ::CmdToCv;
using CmdToEc = ::CmdToEc;
using RobotStatus = ::RobotStatus;
using CvStatus = ::CvStatus;
}  // namespace rmcv::communicate

#endif