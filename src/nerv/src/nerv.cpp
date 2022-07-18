#include "nerv/nerv.hpp"

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

namespace nerv
{
namespace __impl
{
YAML::Node config_;
}

void init(const std::string& config_path)
{
  static int init_done = 0;
  if (init_done) NERV_WARN("重复初始化 NERV");
  init_done = 1;

  __impl::config_ = YAML::LoadFile(config_path);
}

}  // namespace nerv
