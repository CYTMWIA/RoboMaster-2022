#ifndef __RM_NODE_DETECT_THREAD_HPP__
#define __RM_NODE_DETECT_THREAD_HPP__

/*
 * 目标检测结果名称为 "detect_result"
 */

#include <memory>
#include <thread>

#include "base_thread.hpp"
#include "rm_config/config.hpp"
#include "rm_detect/detect.hpp"

namespace rm_node
{
DECL_WORKTHTREAD(DetectThread)

private:
rm_detect::CvArmorDetector detector;
// rm_detect::NnArmorDetector detector;

public:
DetectThread(const rm_config::Config &cfg);

DECL_WORKTHTREAD_END()
}  // namespace rm_node

#endif