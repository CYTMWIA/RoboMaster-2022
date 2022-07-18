#ifndef __NERV_NERV_BASE_HPP__
#define __NERV_NERV_BASE_HPP__

#include <yaml-cpp/yaml.h>

#include <boost/lexical_cast.hpp>
#include <cstddef>
#include <exception>
#include <string>

#include "nerv/logging.hpp"

namespace nerv
{
namespace __impl
{
extern YAML::Node config_;

template <typename Tp, typename Tp2>
Tp __get_parameter(const YAML::Node& node, Tp2&& fallback)
{
  if (node)
    return node.as<Tp>();
  else
    return (Tp)fallback;
}

template <typename Tp, typename Tp2, typename... Args>
Tp __get_parameter(const YAML::Node& node, Tp2&& key, Args&&... keys)
{
  return __get_parameter<Tp>(node[key], keys...);
}

template <typename T>
constexpr inline decltype(auto) __parameter_pack_last(T&& t)
{
  return std::forward<T>(t);
}

template <typename... Args>
decltype(auto) parameter_pack_last(Args&&... args)
{
  // https://stackoverflow.com/questions/61321031/get-last-element-of-parameter-pack-in-c17-c20
  return (__parameter_pack_last(std::forward<Args>(args)), ...);
}

template <typename Tp, typename Tp2>
std::string concat_keys_string(Tp&& key, Tp2&& last_arg_is_not_key)
{
  (void)last_arg_is_not_key;  // Avoid warning: unused parameter
  return std::string(key);
}

template <typename Tp, typename... Args>
std::string concat_keys_string(Tp&& key, Args&&... args)
{
  return std::string(key) + ", " + concat_keys_string(args...);
}

template <typename Tp>
std::string to_string(Tp&& key)
{
  return boost::lexical_cast<std::string>(key);
}

template <typename Tp>
std::string to_string(const std::vector<Tp>& vec)
{
  std::string res = "[";
  if (vec.size()) res += to_string(vec[0]);
  for (size_t i = 1; i < vec.size(); i++) res += ", " + to_string(vec[i]);
  res += "]";
  return res;
}

}  // namespace __impl

/**
 * @brief 读取配置文件中的参数
 *
 * @tparam Tp 参数类型
 * @tparam Args
 * @param keys 键值
 * @return Tp 参数值
 */
template <typename Tp, typename... Args>
Tp get_parameter(Args&&... args)
{
  try
  {
    Tp value = __impl::__get_parameter<Tp>(__impl::config_, args...);
    NERV_INFO("Get parameter ({}): {}", __impl::concat_keys_string(args...),
              __impl::to_string(value));
    return value;
  }
  catch (const std::exception& ex)
  {
    NERV_WARN(
        "\n"
        "Get parameter ({}) failed: {}\n"
        "Use fallback value: {}",
        __impl::concat_keys_string(args...), ex.what(),
        __impl::to_string((Tp)__impl::parameter_pack_last(args...)));
    return (Tp)__impl::parameter_pack_last(args...);
  }
}

/**
 * @brief 初始化 NERV
 *
 * @param config_path 配置文件路径
 */
void init(const std::string& config_path);

}  // namespace nerv
#endif