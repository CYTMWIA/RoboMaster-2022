#ifndef __UNIFORMLINEARMOTIONR_HPP__
#define __UNIFORMLINEARMOTIONR_HPP__

/* 匀速直线运动模型
 * 适用于 相机坐标系 && AdaptiveEKF
 * 状态向量：x, x_speed, y, y_speed, z, z_speed
 * 测量向量：x, y, z
 */

namespace rmcv::predict::ulmr
{
    /**
     * @brief 代替状态转移矩阵
     * 
     */
    struct Predict
    {
        template <class T>
        void operator()(const T x0[6], T x1[6])
        {
            x1[0] = x0[0] + delta_t * x0[1];
            x1[1] = x0[1];
            x1[2] = x0[2] + delta_t * x0[3];
            x1[3] = x0[3];
            x1[4] = x0[4] + delta_t * x0[5];
            x1[5] = x0[5];
        }

        double delta_t;
    };

    /**
     * @brief 于将 状态向量 转为与 测量值 相同意义的矩阵（向量）
     * 
     */
    struct Measure
    {
        template <class T>
        void operator()(const T x[6], T y[3])
        {
            y[0] = x[0];
            y[1] = x[2];
            y[2] = x[4];
        }
    };
}

#endif
