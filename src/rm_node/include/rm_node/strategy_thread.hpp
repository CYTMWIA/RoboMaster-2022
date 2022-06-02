#ifndef __RM_NODE_STRATEGY_THREAD_HPP__
#define __RM_NODE_STRATEGY_THREAD_HPP__

#include "base_thread.hpp"
#include "rm_common/filter.hpp"
#include "rm_config/config.hpp"
#include "rm_predict/predict.hpp"
#include "ulm_xyz_kf.hpp"

namespace rm_node
{
DECL_WORKTHTREAD(StrategyThread)
private:
rm_filter::spec::UlmXyzKf kf_;
rm_predict::PnpSolver pnp_solver_;

public:
StrategyThread(const rm_config::Config &cfg);

DECL_WORKTHTREAD_END()
}  // namespace rm_node

#endif