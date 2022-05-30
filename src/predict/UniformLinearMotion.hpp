#ifndef __UNIFORMLINEARMOTION_HPP__
#define __UNIFORMLINEARMOTION_HPP__

/* 匀速直线运动模型
 * 适用于 世界坐标系 && AdaptiveEKF
 * 状态向量：x, x_speed, y, y_speed, z
 * 测量向量：pitch, yaw, distance
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
        void operator()(const T x0[5], T x1[5])
        {
            x1[0] = x0[0] + delta_t * x0[1];
            x1[1] = x0[1];
            x1[2] = x0[2] + delta_t * x0[3];
            x1[3] = x0[3];
            x1[4] = x0[4];
        }

        double delta_t;
    };

    template <class T>
    void xyz2pyd(T xyz[3], T pyd[3])
    {
        /*
         * 工具函数：将 xyz 转化为 pitch、yaw、distance
         */
        pyd[0] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
        pyd[1] = ceres::atan2(xyz[1], xyz[0]);                                         // yaw
        pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);     // distance
    }

    /**
     * @brief 于将 状态向量 转为与 测量值 相同意义的矩阵（向量）
     * 
     */
    struct Measure
    {
        template <class T>
        void operator()(const T x[5], T y[3])
        {
            T x_[3] = {x[0], x[2], x[4]};
            xyz2pyd(x_, y);
        }
    };
}

#endif
