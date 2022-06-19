#ifndef __WORK_THREAD_DETECT_THREAD_HPP__
#define __WORK_THREAD_DETECT_THREAD_HPP__

/*
 * 目标检测结果名称为 "detect_result"
 */

#include <memory>
#include <thread>

#include "base_thread.hpp"
#include "config/config.hpp"
#include "detect/detect.hpp"

namespace rm_work_thread
{
DECL_WORKTHTREAD(DetectThread)

private:
rm_detect::OcvArmorDetector armor_detector;
rm_detect::OcvPowerRuneDetector power_rune_detector;
// rm_detect::NnArmorDetector armor_detector;

public:
DetectThread(const rm_config::Config &cfg);

DECL_WORKTHTREAD_END()
}  // namespace rm_work_thread

#endif