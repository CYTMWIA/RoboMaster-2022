#include "UlmXyzKf.hpp"

#include <iostream>

namespace rmcv::filter::spec
{
    UlmXyzKf::UlmXyzKf()
    {
        Q = MatrixXX::Identity();
        R = MatrixYY::Identity();
        F = MatrixXX::Identity();
        H << 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0;
    }

    void UlmXyzKf::init(double x, double y, double z)
    {
        last_predict_ = std::chrono::steady_clock::now();

        X << x, 0, y, 0, z, 0;
        P = MatrixXX::Identity();
    }

    void UlmXyzKf::predict(double dt)
    {
        auto this_time = std::chrono::steady_clock::now();
        if (dt < 0)
            dt = std::chrono::duration_cast<std::chrono::microseconds>(this_time - last_predict_).count() / 1000000.0;
        last_predict_ = this_time;

        F(0, 1) = F(2, 3) = F(4, 5) = dt;
        KalmanFilter<6, 3>::predict();
    }

    KalmanFilter<6, 3>::VectorX UlmXyzKf::predict_without_save(double dt)
    {
        F(0, 1) = F(2, 3) = F(4, 5) = dt;
        return F * X;
    }

    void UlmXyzKf::update(double x, double y, double z)
    {
        last_predict_ = std::chrono::steady_clock::now();

        Eigen::Matrix<double, 3, 1> yr;
        yr << x, y, z;
        KalmanFilter<6, 3>::update(yr);
    }
}