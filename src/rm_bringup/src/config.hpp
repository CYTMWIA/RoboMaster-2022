#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include <string>
#include <toml.hpp>
#include <type_traits>
#include <vector>

namespace rm_bringup
{
namespace config
{
using Integer = toml::integer;
using Float = toml::floating;
using Bool = toml::boolean;
using String = toml::string;

toml::value RM_CONFIG;
std::string RM_CONFIG_PATH{"asset/config.toml"};

void init(const std::string& path = RM_CONFIG_PATH) { RM_CONFIG = toml::parse(path); }

/* 模板类特化检查
 * https://stackoverflow.com/questions/16337610/how-to-know-if-a-type-is-a-specialization-of-stdvector
 * https://stackoverflow.com/questions/9392777/enable-template-only-for-some-stdvectort2-type
 */

template <typename T>
void __print_config_recursion(const std::vector<T>& last)
{
  std::cout << " = ";
  std::cout << "[ ";
  for (const auto& e : last) std::cout << e << " ";
  std::cout << "]" << std::endl;
}

template <typename T>
void __print_config_recursion(const T& last)
{
  std::cout << " = " << last << std::endl;
}

template <typename T, typename... Args>
void __print_config_recursion(const T& first, Args&&... args)
{
  std::cout << "." << first;
  __print_config_recursion(args...);
}

template <typename T, typename... Args>
void __print_config(const T& first, Args&&... args)
{
  std::cout << "[读取配置] " << first;
  __print_config_recursion(args...);
}

template <typename T, typename... Args>
T get(T fallback, Args&&... args)
{
  T cfg = toml::find_or<T>(RM_CONFIG, args..., (T)fallback);
  __print_config(args..., cfg);
  return cfg;
}
}  // namespace config
}  // namespace rm_bringup

#endif