#ifndef __FILTER_SPEC_ULMXYZKF_HPP__
#define __FILTER_SPEC_ULMXYZKF_HPP__

/*
 * 基于XYZ坐标系的匀速直线运动模型普通卡尔曼滤波
 */

#include "../KalmanFilter.hpp"

#include <chrono>

namespace rmcv::filter::spec
{
    class UlmXyzKf : public KalmanFilter<6, 3>
    {
    private:
        std::chrono::time_point<std::chrono::steady_clock> last_predict_;

    public:
        UlmXyzKf();

        void init(double x, double y, double z);

        void predict(double dt = -1);

        void update(double x, double y, double z);

        inline double x() { return X[0]; }
        inline double y() { return X[2]; }
        inline double z() { return X[4]; }
    };
}

#endif // __FILTER_SPEC_ULMXYZKF_HPP__