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
        float tx = target_pos(0, 0), ty = target_pos(1, 0), tz = target_pos(2, 0);

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

        float a = (-g_ * x * x) / (2 * bullet_speed_*1000 * y), b = x / y;
        float d = sqrt(b * b - 4 * a * (a - 1));
        if (d < 0) // 无解，放弃
        {
            __LOG_DEBUG("无解");
            return AimDeviation(0, 0);
        }
        __LOG_DEBUG("{}, {}, {}, {}, {}", x, y, a, b, d);
        // 选择接近零的结果（角度小）
        float tan1 = (-b + d) / (2 * a);
        float tan2 = (-b - d) / (2 * a);
        float theta = atan((abs(tan1) < abs(tan2)) ? tan1 : tan2);

        float pitch = theta - real_pitch;

        return AimDeviation(yaw, pitch);
    }
}