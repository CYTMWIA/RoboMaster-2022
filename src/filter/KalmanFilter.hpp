#ifndef __FILTER_KALMANFILTER_HPP__
#define __FILTER_KALMANFILTER_HPP__

#include <Eigen/Dense>

namespace rmcv::filter
{

    template <int N_X, int N_Y>
    class KalmanFilter
    {
    public:
        using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
        using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
        using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
        using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
        using VectorX = Eigen::Matrix<double, N_X, 1>;
        using VectorY = Eigen::Matrix<double, N_Y, 1>;
    
        VectorX X;
        MatrixXX F;
        MatrixXX P;
        MatrixXX Q;
        MatrixYX H;
        MatrixYY R;

        void init(const VectorX &x_in = VectorX::Zero())
        {
            X = x_in;
        }

        // template <typename T>
        // void set_q_diag(std::vector<T> vec)
        // {
        //     for (int i = 0; i < N_X; i++)
        //         Q(i, i) = vec[i];
        // }

        // template <typename T>
        // void set_r_diag(std::vector<T> vec)
        // {
        //     for (int i = 0; i < N_Y; i++)
        //         R(i, i) = vec[i];
        // }

        void predict()
        {
            X = F * X;
            MatrixXX Ft = F.transpose();
            P = F * P * Ft + Q;
        }

        void update(const VectorY &z)
        {
            VectorY y = z - H * X;
            MatrixXY Ht = H.transpose();
            MatrixXY K = P * Ht * ((H * P * Ht) + R).inverse();
            X = X + (K * y);
            P = (MatrixXX::Identity() - K * H) * P;
        }
    };

} // namespace rmcv::filter

#endif // __FILTER_KALMANFILTER_HPP__