#ifndef __WORK_THREAD_STRATEGYTHREAD_HPP__
#define __WORK_THREAD_STRATEGYTHREAD_HPP__

#include "BaseThread.hpp"

#include "config/config.hpp"
#include "predict/predict.hpp"

namespace rmcv::work_thread
{
    DECL_WORKTHTREAD(StrategyThread)
private:
    AdaptiveEKF<6, 3> ekf_;
    rmcv::predict::PnpSolver pnp_solver_;
public:
    StrategyThread(const rmcv::config::Config &cfg);

    DECL_WORKTHTREAD_END()
}

#endif