#ifndef __AIMER_HPP__
#define __AIMER_HPP__

#include "rm_data.hpp"

#include <Eigen/Dense>

namespace rmcv::predict
{
    /**
     * @brief 瞄准偏差
     *
     */
    struct AimResult
    {
        bool ok;
        float yaw;
        float pitch;
        float flying_time;

        AimResult() = default;
        AimResult(decltype(ok) _ok, decltype(yaw) _yaw=0, decltype(pitch) _pitch=0, decltype(flying_time) _flying_time = 0);
    };

    class Aimer
    {
    private:
        const double G = 0.0098;         // 重力加速度，单位：mm/(ms^2)
        const double Rho_Air = 1.184e-9; // 空气密度，单位：kg/mm^3
        const double Cd_Sphere = 0.042;  // 阻力系数
        const double dt = 0.1;           // 计算时间间隔，单位：ms

        double bullet_speed_ = 0.016;                                                                                 // 子弹初速度，单位：mm/(ms^2)
        double bullet_area_ = 2 * M_PI * (rm_data::BULLET_SMALL_DIAMETER / 2) * (rm_data::BULLET_SMALL_DIAMETER / 2); // 子弹横截面积，单位：mm^2

        struct TestResult
        {
            bool ok;
            double deviation;
            double time;
        };
        TestResult test(double target_x, double target_y, double rad, double overtime = 600);

    public:
        Aimer() = default;

        void bullet_speed(double bs);
        void bullet_type(rm_data::BulletType bullet_type);

        /**
         * @brief 瞄准静态目标
         *
         * @param target_pos 目标相对于相机的坐标
         * @param real_pitch 目前机器人实际pitch轴角度
         * @return AimResult
         */
        AimResult operator()(const Eigen::Matrix<double, 3, 1> &target_pos, float real_pitch = 0);
    };
}

#endif