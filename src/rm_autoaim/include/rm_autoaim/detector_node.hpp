#pragma once

#include "nerv/nerv.hpp"

namespace rm_autoaim
{
class DetectorNode : public nerv::Node
{
 public:
  DetectorNode();
  void run() override;
};
}  // namespace rm_autoaim
