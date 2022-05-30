#ifndef __FILTER_DYNAMICLOWPASSFILTER_HPP__
#define __FILTER_DYNAMICLOWPASSFILTER_HPP__

#include <Eigen/Dense>
#include <algorithm>

/*
 * 未完成，不建议使用（直接用卡尔曼吧
 */

namespace rmcv::filter
{
    template<int N_X>
    class DynamicLowpassFilter
    {
        using VectorX = Eigen::Matrix<double, N_X, 1>;
    public:
        VectorX output;

        VectorX init(const VectorX &data)
        {
            return output = data;
        }

        VectorX update(const VectorX &data)
        {
            using namespace std;

            VectorX diff;
            for (int i=0;i<N_X;i++)
                if (output[i] != 0 && data[i] != 0) diff[i] = max(abs(data[i]), abs(output[i]))/min(abs(data[i]), abs(output[i])) - 1;
                else diff[i] = 1;
            
            // cout << "------ diff\n" << diff << endl;

            // cout << "------ conf ";
            for (int i=0;i<N_X;i++)
            {
                double last_conf = (tanh((diff[i] + 4.5) * 0.01) + 1.0) / 2.0;
                // if (diff[i] >= 100) last_conf = 1;
                // cout << last_conf << ", ";
                output[i] = last_conf * output[i] + (1 - last_conf) * data[i];
            }
            // cout << endl;

            // cout << "------ out\n" << output << endl;

            return output;
        }
    };
}

#endif // __FILTER_DYNAMICLOWPASSFILTER_HPP__