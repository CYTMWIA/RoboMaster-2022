#include "FpsCounter.hpp"

namespace rmcv::util
{
    FpsCounter::FpsCounter() : tick_time_(std::chrono::steady_clock::now())
    {
    }

    float FpsCounter::tick()
    {
        float fps = (float)(std::chrono::milliseconds(1000) / (std::chrono::steady_clock::now() - tick_time_));
        tick_time_ = std::chrono::steady_clock::now();
        return fps;
    }
}
