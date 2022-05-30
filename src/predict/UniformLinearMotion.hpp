#ifndef __UNIFORMLINEARMOTION_HPP__
#define __UNIFORMLINEARMOTION_HPP__

/* 匀速直线运动模型
 * 适用于 AdaptiveEKF
 * 状态向量：x, x_speed, y, y_speed, z, z_speed
 * 测量向量：pitch, yaw, distance
 * 为什么测量向量要如此设置？详见笔记《为什么上交EKF使用pitch、yaw、distance作为输入》
 */

namespace rmcv::predict::ulm
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

    template <class T>
    void xyz2pyd(T xyz[3], T pyd[3])
    {
        /*
         * 工具函数：将 xyz 转化为 pitch、yaw、distance
         */
        pyd[0] = atan(xyz[2] / sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]));    // pitch
        pyd[1] = -atan(xyz[0] / xyz[1]);                                    // yaw
        pyd[2] = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]); // distance
    }

    /**
     * @brief 于将 状态向量 转为与 测量值 相同意义的矩阵（向量）
     *
     */
    struct Measure
    {
        template <class T>
        void operator()(const T x[6], T y[3])
        {
            T x_[3] = {x[0], x[2], x[4]};
            xyz2pyd(x_, y);
        }
    };
}

#endif
