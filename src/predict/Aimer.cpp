#include <cmath>

#include "logging.hpp"

#include "Aimer.hpp"

namespace rmcv::predict
{
    Aimer::Aimer(float g) : g_(g) {}

    void Aimer::bullet_speed(float bs) { bullet_speed_ = bs; }
    float Aimer::bullet_speed() { return bullet_speed_; }

    AimDeviation Aimer::aim_static(const Eigen::Matrix<double, 3, 1> &target_pos, float real_pitch)
    {
        float tx = target_pos(0, 0), ty = target_pos(1, 0), tz = target_pos(2, 0)+55;

        // Yaw 轴偏差
        float yaw = -atan(tx / ty);

        // Pitch 轴偏差（枪口仰角）
        // 计算过程还请看笔记《弹丸运动学》
        // 将xOy平面转到水平面
        float dis = sqrt(ty * ty + tz * tz), target_pitch = atan(tz/ty)+real_pitch;
        float hx = tx, hy = dis * cos(target_pitch), hz = dis * sin(target_pitch);
        // 假设现在是竖直平面，平面上有【枪口】与【目标】两个点
        float x = sqrt(hx * hx + hy * hy); // 与目标在x轴（横轴）的距离
        float y = hz;                      // 与目标在y轴（纵轴）的距离

        #define __YP(t) ((-g_*x*x)/(2*bullet_speed_*bullet_speed_*cos(t)*cos(t))+tan(t)*x)

        if (__YP(M_PI/4.0)<y) return AimDeviation(0, 0);

        // 二分法
        float lo = -M_PI/4.0, hi = M_PI/4.0, mid=0, last_mid=999;
        while (abs(mid-last_mid) > 0.001)
        {
            float yp = __YP(mid);
            
            if (abs(yp-y)<1) break;

            if (yp>y) hi = mid;
            else lo = mid;

            last_mid = mid;
            mid = (lo+hi)/2.0;
        }
        float pitch = mid - real_pitch;

        #undef __YP

        return AimDeviation(yaw, pitch);
    }
}