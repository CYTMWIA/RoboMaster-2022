#ifndef __PREDICT_ULM_XYZ_KF_HPP__
#define __PREDICT_ULM_XYZ_KF_HPP__

/*
 * 基于XYZ坐标系的匀速直线运动模型普通卡尔曼滤波
 */

#include <chrono>

#include "rm_common/filter/kalman_filter.hpp"

namespace rm_autoaim
{
class UlmXyzKf : public rm_filter::KalmanFilter<6, 3>
{
 private:
  std::chrono::time_point<std::chrono::steady_clock> last_predict_;

 public:
  UlmXyzKf();

  void init(double x, double y, double z);

  void predict(double dt = -1);

  KalmanFilter<6, 3>::VectorX predict_without_save(double dt);

  void update(double x, double y, double z);

  inline double x() { return X[0]; }
  inline double y() { return X[2]; }
  inline double z() { return X[4]; }
};
}  // namespace rm_autoaim

#endif  // __FILTER_SPEC_ULMXYZKF_HPP__