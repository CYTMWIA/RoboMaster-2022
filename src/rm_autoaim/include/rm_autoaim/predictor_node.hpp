#pragma once

#include "nerv/nerv.hpp"

namespace rm_autoaim
{
class PredictorNode : public nerv::Node
{
 public:
  void run() override;
};
}  // namespace rm_autoaim