#ifndef __UTIL_MOVINGAVERAGEFILTER_HPP__
#define __UTIL_MOVINGAVERAGEFILTER_HPP__

#include <vector>

namespace rmcv::util
{
    class MovingAverageFilter
    {
    private:
        int history_size_;
        std::vector<double> history_;
        int idx_;

    public:
        MovingAverageFilter(int n = 5);
        double update(double data);
        double value();
    };
}

#endif