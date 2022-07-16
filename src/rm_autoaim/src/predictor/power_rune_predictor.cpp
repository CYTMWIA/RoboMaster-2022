#include "power_rune_predictor.hpp"

#include <chrono>

#include "common/logging.hpp"
#include "common/threading/roslike_topic.hpp"

namespace rm_autoaim
{

void PowerRunePredictor::clear_expired_history()
{
  if (history_.empty()) return;

  if (rmc::interfaces::now() - history_[history_.size() - 1].image_timestamp >
      std::chrono::milliseconds(500))
    history_.clear();
}

void PowerRunePredictor::predict(const DetectResult<PowerRune>& detect, int power_rune_type)
{
  clear_expired_history();

  if (detect.content.size())
  {
    HistoryFrame frame = {.image_timestamp = detect.image_timestamp, .power_runes = detect.content};
    if (history_.size())
    {
      auto& last_power_runes = history_[history_.size() - 1].power_runes;
      for (int i = 0; i < (int)last_power_runes.size() && i < (int)frame.power_runes.size(); i++)
      {
        std::sort(frame.power_runes.begin() + i, frame.power_runes.end(),
                  [&](const auto& a, const auto& b)
                  {
                    return distance(a.armor_center, last_power_runes[i].armor_center) <
                           distance(b.armor_center, last_power_runes[i].armor_center);
                  });
      }
    }
    history_.push_back(std::move(frame));
  }

  if (history_.size() >= 10)
  {
    double speed = 0;
    for (int i = 0; i < 3; i++)
    {
      auto& last1 = history_[history_.size() - 1 - i];
      auto& last2 = history_[history_.size() - 10 + i];

      auto angle = included_angle(last1.power_runes[0].logo, last2.power_runes[0].armor_center,
                                  last1.power_runes[0].armor_center);
      auto rad = to_rad(angle);
      auto time = std::chrono::duration_cast<std::chrono::microseconds>(last1.image_timestamp -
                                                                        last2.image_timestamp)
                      .count() /
                  1000000.0;
      speed += rad / time;
    }
    speed /= 3.0;
    rm_threading::RoslikeTopic<std::vector<float>>::set(
        "vofa_justfloat",
        {(float)(std::chrono::duration_cast<std::chrono::milliseconds>(history_[history_.size() - 5].image_timestamp.time_since_epoch()).count()),
         (float)speed});
    __LOG_DEBUG("{}", speed);
  }


}

}  // namespace rm_autoaim