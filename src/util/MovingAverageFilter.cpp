#include "MovingAverageFilter.hpp"

namespace rmcv::util
{
    MovingAverageFilter::MovingAverageFilter(int n) : history_size_(n), history_(n), idx_(0)
    {
    }

    double MovingAverageFilter::update(double data)
    {
        history_[idx_] = data;
        idx_ = (idx_+1)%history_size_;

        return value();
    }

    double MovingAverageFilter::value()
    {
        double res = 0;
        for (const auto& v: history_) res += v;
        return res/history_size_;
    }
}
