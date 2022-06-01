#ifndef __FILTER_MOVING_AVERAGE_FILTER_HPP__
#define __FILTER_MOVING_AVERAGE_FILTER_HPP__

#include <algorithm>
#include <vector>

namespace rm_filter
{
template <int N_X, int N_HISTORY>
class MovingAverageFilter
{
  using VectorX = Eigen::Matrix<double, N_X, 1>;
  using MatrixXH = Eigen::Matrix<double, N_X, N_HISTORY>;

 private:
  MatrixXH history_;
  int idx_;

 public:
  VectorX output;

  MovingAverageFilter() : idx_(0) {}

  VectorX init(const VectorX &data)
  {
    for (int i = 0; i < N_X; i++)
      for (int j = 0; j < N_HISTORY; j++) history_(i, j) = data[i];

    for (int i = 0; i < N_X; i++)
    {
      output[i] = 0;
      for (int j = 0; j < N_HISTORY; j++) output[i] += history_(i, j);
      output[i] /= 1.0 * N_HISTORY;
    }

    return output;
  }

  VectorX update(const VectorX &data)
  {
    using namespace std;

    for (int i = 0; i < N_X; i++) history_(i, idx_) = data[i];
    idx_ = (idx_ + 1) % N_HISTORY;

    for (int i = 0; i < N_X; i++)
    {
      output[i] = 0;
      for (int j = 0; j < N_HISTORY; j++) output[i] += history_(i, j);
      output[i] = output[i] / N_HISTORY;
    }

    return output;
  }
};
}  // namespace rm_filter

#endif