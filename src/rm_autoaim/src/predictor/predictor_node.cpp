#include "rm_autoaim/predictor_node.hpp"

#include <chrono>

#include "../interfaces.hpp"
#include "nerv/nerv.hpp"
#include "power_rune_predictor.hpp"

namespace rm_autoaim
{
void PredictorNode::run()
{
  PowerRunePredictor power_rune_predictor;
  auto start_time = nerv::interfaces::now();
  while (true)
  {
    auto dr = nerv::Topic<DetectResult<PowerRune>>::get("DetectResult/PowerRune");
    power_rune_predictor.predict(dr, 0);
  }
}
}  // namespace rm_autoaim