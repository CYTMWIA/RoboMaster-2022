#ifndef __RM_SERIAL_SERIAL_NODE_HPP__
#define __RM_SERIAL_SERIAL_NODE_HPP__

#include "nerv/nerv.hpp"

namespace rm_serial
{

class SerialNode : public nerv::Node
{
 private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;

 public:
  SerialNode();
  ~SerialNode();

  void run() override;
};

}  // namespace rm_serial
#endif