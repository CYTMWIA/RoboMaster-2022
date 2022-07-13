#ifndef __COMMON_ASSERT_HPP__
#define __COMMON_ASSERT_HPP__

#include <iostream>

#define RM_ASSERT(msg, x)                                                         \
  if (!(x))                                                                       \
  {                                                                               \
    std::cout << "Assert Error At: " << __FILE__ << ":" << __LINE__ << std::endl; \
    std::cout << msg << ": " << #x << std::endl;                                  \
    exit(1);                                                                      \
  }

#endif
