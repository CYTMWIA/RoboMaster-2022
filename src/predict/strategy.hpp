#ifndef __STRATEGY_HPP__
#define __STRATEGY_HPP__

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "threading.hpp"

namespace rmcv
{
    void start_strategy(const Config::Strategy& cfg);
}

#endif