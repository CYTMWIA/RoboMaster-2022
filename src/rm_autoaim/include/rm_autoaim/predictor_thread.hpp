#ifndef __RM_AUTOAIM_PREDICTOR_THREAD_HPP__
#define __RM_AUTOAIM_PREDICTOR_THREAD_HPP__

#include "common/threading.hpp"

namespace rm_autoaim
{
class PredictorThread : public rm_common::BaseThread
{
 public:
  void run() override;
};
}  // namespace rm_autoaim

#endif