#ifndef __WORK_THREAD_STRATEGY_THREAD_HPP__
#define __WORK_THREAD_STRATEGY_THREAD_HPP__

#include "base_thread.hpp"
#include "config/config.hpp"
#include "filter/filter.hpp"
#include "predict/predict.hpp"

namespace rmcv::work_thread
{
DECL_WORKTHTREAD(StrategyThread)
private:
rmcv::filter::spec::UlmXyzKf kf_;
rmcv::predict::PnpSolver pnp_solver_;

public:
StrategyThread(const rmcv::config::Config &cfg);

DECL_WORKTHTREAD_END()
}  // namespace rmcv::work_thread

#endif