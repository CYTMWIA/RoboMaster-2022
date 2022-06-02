#ifndef __RM_COMMUNICATE_SERIAL_HPP__
#define __RM_COMMUNICATE_SERIAL_HPP__

#include <memory>
#include <string>
#include <vector>

#include "cmd.hpp"
#include "vofa.hpp"

namespace rm_communicate
{
class Serial
{
 private:
  class Impl;
  std::unique_ptr<Impl> pimpl;

 public:
  Serial(std::string dev = "/dev/ttyUSB0", uint32_t baud_rate = 115200);
  ~Serial();

  void send(const CvStatus &status);
  void update(RobotStatus &status);
};
}  // namespace rm_communicate

#endif