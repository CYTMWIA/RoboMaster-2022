#ifndef __AIMER_HPP__
#define __AIMER_HPP__

#include <Eigen/Dense>

namespace rmcv::predict
{
    /**
     * @brief 瞄准偏差
     *
     */
    struct AimDeviation
    {
        float yaw;
        float pitch;

        template <typename T>
        AimDeviation(T &&_yaw, T &&_pitch) : yaw(_yaw), pitch(_pitch)
        {
        }
    };

    class Aimer
    {
    private:
        float g_;                    // 重力加速度，单位：mm/s^2
        float bullet_speed_ = 15000; // 子弹初速度，单位：mm/s
    public:
        Aimer(float g = 9800);

        void bullet_speed(float bs);
        float bullet_speed();

        /**
         * @brief 瞄准静态目标
         *
         * @param target_pos 目标相对于相机的坐标
         * @param real_pitch 目前机器人实际pitch轴角度
         * @return AimDeviation
         */
        AimDeviation aim_static(const Eigen::Matrix<double, 3, 1> &target_pos, float real_pitch = 0);
    };
}

#endif