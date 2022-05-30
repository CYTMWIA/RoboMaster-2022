#ifndef __COMMUNICATE_HPP__
#define __COMMUNICATE_HPP__

#include "config.hpp"
#include "threading.hpp"

namespace rmcv
{
    void start_communicate(const Config::Communicate& cfg);
}

#endif