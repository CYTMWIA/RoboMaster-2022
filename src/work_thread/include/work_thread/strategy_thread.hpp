#ifndef __WORK_THREAD_STRATEGY_THREAD_HPP__
#define __WORK_THREAD_STRATEGY_THREAD_HPP__

#include "base_thread.hpp"
#include "common/filter.hpp"
#include "config/config.hpp"
#include "predict/predict.hpp"
#include "ulm_xyz_kf.hpp"

namespace rm_work_thread
{
DECL_WORKTHTREAD(StrategyThread)
private:
rm_filter::spec::UlmXyzKf kf_;
rm_predict::PnpSolver pnp_solver_;

public:
StrategyThread(const rm_config::Config &cfg);

DECL_WORKTHTREAD_END()
}  // namespace rm_work_thread

#endif