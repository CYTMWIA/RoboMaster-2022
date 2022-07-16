#ifndef __PREDICTOR_POWER_RUNE_PREDICTOR_HPP__
#define __PREDICTOR_POWER_RUNE_PREDICTOR_HPP__

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <vector>

#include "../geometry/utils.hpp"
#include "../interfaces.hpp"
#include "common/interfaces.hpp"

namespace rm_autoaim
{
class PowerRunePredictor
{
 private:
  struct HistoryFrame
  {
    rmc::interfaces::Timestamp image_timestamp;
    std::vector<PowerRune> power_runes;
  };
  std::vector<HistoryFrame> history_;
  void clear_expired_history();

 public:
  void predict(const DetectResult<PowerRune>& detect, int power_rune_type);
};
}  // namespace rm_autoaim

#endif