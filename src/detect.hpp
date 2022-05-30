#ifndef __RMCV_DETECT_HPP__
#define __RMCV_DETECT_HPP__

#include "config/config.hpp"

/*
 * 目标检测结果名称为 "detect/result"
 */

namespace rmcv
{
   void thread_detect(const config::Config &cfg);
}

#endif