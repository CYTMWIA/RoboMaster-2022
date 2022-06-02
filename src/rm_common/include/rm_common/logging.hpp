#ifndef __RM_COMMON_LOGGING_HPP__
#define __RM_COMMON_LOGGING_HPP__

#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/printf.h>

#include <chrono>
#include <string_view>

#define __LOG_CURRENT_TIME fmt::localtime(std::time(nullptr))

constexpr std::string_view basename_constexpr(std::string_view path)
{
  /* 参考资料：
   * https://stackoverflow.com/questions/27123306/is-it-possible-to-use-stdstring-in-a-constexpr
   * https://stackoverflow.com/questions/66126750/trim-both-path-and-extension-from-file-compile-time
   * 不能把__FILE__宏直接置于此函数内，那会导致return结果为此hpp文件名，而非实际调用所在的文件名
   */
  return path.substr(path.find_last_of('/') + 1,
                     path.find_last_of('.') - path.find_last_of('/') - 1);
}

#define __LOG(type_name, color, fmsg, ...)                                             \
  fmt::print(color, "[{:%H:%M:%S}][" type_name "][{}] " fmsg "\n", __LOG_CURRENT_TIME, \
             basename_constexpr(__FILE__), ##__VA_ARGS__)

#define __LOG_ERROR(fmsg, ...) __LOG("ERROR", fg(fmt::color::red), fmsg, ##__VA_ARGS__)

#define __LOG_WARNING(fmsg, ...) __LOG("WARNING", fg(fmt::color::yellow), fmsg, ##__VA_ARGS__)

#define __LOG_INFO(fmsg, ...) __LOG("INFO", fg(fmt::color::white), fmsg, ##__VA_ARGS__)

#define __LOG_DEBUG(fmsg, ...) __LOG("DEBUG", fg(fmt::color::gray), fmsg, ##__VA_ARGS__)

#define __LOG_ERROR_AND_EXIT(fmsg, ...) \
  {                                     \
    __LOG_ERROR(fmsg, ##__VA_ARGS__);   \
    exit(1);                            \
  }

#endif