#ifndef __UTIL_UTIL_HPP__
#define __UTIL_UTIL_HPP__

#include "FpsCounter.hpp"

namespace rmcv::util
{
    template<typename T, typename A, typename B>
    T limit(T val, A min, B max)
    {
        if (val>max) return max;
        else if (val<min) return min;
        else return val;
    }
}

#endif