#ifndef __WORK_THREAD_DETECTTHREAD_HPP__
#define __WORK_THREAD_DETECTTHREAD_HPP__

/*
 * 目标检测结果名称为 "detect_result"
 */

#include "BaseThread.hpp"

#include "config/config.hpp"
#include "detect/detect.hpp"

#include <thread>
#include <memory>

namespace rmcv::work_thread
{
    DECL_WORKTHTREAD(DetectThread)

private:
    rmcv::detect::CvArmorDetector detector;
    // rmcv::detect::NnArmorDetector detector;

public:
    DetectThread(const rmcv::config::Config &cfg);

    DECL_WORKTHTREAD_END()
}

#endif