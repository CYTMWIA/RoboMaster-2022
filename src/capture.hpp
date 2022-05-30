#ifndef __RMCV_CAPTURE_HPP__
#define __RMCV_CAPTURE_HPP__

#include "config/config.hpp"

/*
 * 捕获到的图像名称为 "capture_image"
 */

namespace rmcv
{
   void thread_capture(const config::Config &cfg);
}

#endif