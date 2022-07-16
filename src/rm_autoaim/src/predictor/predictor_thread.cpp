#include "rm_autoaim/predictor_thread.hpp"
#include <chrono>

#include "../interfaces.hpp"
#include "common/interfaces.hpp"
#include "common/logging.hpp"
#include "power_rune_predictor.hpp"

namespace rm_autoaim
{
void PredictorThread::run()
{
  PowerRunePredictor power_rune_predictor;
  auto start_time = rmc::interfaces::now();
  while (true)
  {
    auto dr = rm_threading::RoslikeTopic<DetectResult<PowerRune>>::get("DetectResult/PowerRune");
    power_rune_predictor.predict(dr, 0);
  }
}
}  // namespace rm_autoaim