#include "aimer.hpp"

#include <cmath>

namespace rm_autoaim
{
AimResult::AimResult(decltype(ok) _ok, decltype(yaw) _yaw, decltype(pitch) _pitch,
                     decltype(flying_time) _flying_time)
    : ok(_ok), yaw(_yaw), pitch(_pitch), flying_time(_flying_time)
{
}

void Aimer::bullet_speed(double bs) { bullet_speed_ = bs; }
void Aimer::bullet_type(nerv::rm_data::BulletType bullet_type)
{
  using namespace nerv::rm_data;
  if (bullet_type == BULLET_17)
  {
    bullet_area_ = 2 * M_PI * (BULLET_17_DIAMETER / 2) * (BULLET_17_DIAMETER / 2);
    bullet_mass_ = BULLET_17_MASS;
  }
  else
  {
    bullet_area_ = 2 * M_PI * (BULLET_42_DIAMETER / 2) * (BULLET_42_DIAMETER / 2);
    bullet_mass_ = BULLET_42_MASS;
  }
}

Aimer::TestResult Aimer::test(double target_x, double target_y, double rad, double overtime)
{
  TestResult res;

#if 1
  // 无用overtime，消除警告
  while (0 && overtime)
    ;
  // 斜抛公式
  // clang-format off
  res.y_deviation = target_y-((-G*target_x*target_x)/(2*bullet_speed_*bullet_speed_*cos(rad)*cos(rad))+tan(rad)*target_x);
  // clang-format on
  res.time = target_x / (bullet_speed_ * cos(rad));
  res.ok = res.y_deviation <= 0;
  return res;
#else
  // 空气阻力模型
  // 迭代法计算弹道
  double t = 0, x = 0, y = 0, vx = bullet_speed_ * cos(rad), vy = bullet_speed_ * sin(rad);
  while (1)
  {
    t += dt;
    x += vx;
    y += vy;

    // __LOG_DEBUG("{}, {}, {}, {}, {}, {}, {}, {}", rad, target_x, target_y, x, y, vx, vy, t);

    if (x >= target_x)
    {
      res.ok = true;
      double r = 1 - (target_x - (x - vx)) / vx;
      y -= vy * r;
      t -= dt * r;
      break;
    }
    if (t > overtime || (y < target_y && vy < 0))
    {
      res.ok = false;
      break;
    }
    // clang-format off
    vx += (-0.5 * Rho_Air * vx * vx * Cd_Sphere * bullet_area_) / bullet_mass_ * dt;
    vy += (-G * bullet_mass_ - 0.5 * Rho_Air * vy * vy * Cd_Sphere * bullet_area_) / bullet_mass_ * dt;
    // clang-format on
  }
  res.y_deviation = target_y - y;
  res.time = t;
#endif
  return res;
}

AimResult Aimer::operator()(const Eigen::Matrix<double, 3, 1> &target_pos)
{
  float tx = target_pos(0, 0), ty = target_pos(1, 0), tz = target_pos(2, 0);

  // Yaw 轴偏差
  float yaw = atan2(ty, tx);
  // 遵守正前为零点的约定
  if (yaw < -M_PI / 2.0)
    yaw = M_PI + yaw + M_PI / 2.0;
  else
    yaw -= M_PI / 2.0;

  // Pitch 轴偏差（枪口仰角）
  // 计算过程还请看笔记《弹丸运动学》

  // 将xOy平面“转”到水平面
  // float dis = sqrt(ty * ty + tz * tz), target_pitch = atan(tz / ty);
  // float hx = tx, hy = dis * cos(target_pitch), hz = dis * sin(target_pitch);
  // 假设现在是竖直平面，平面上有【枪口】与【目标】两个点
  float x = sqrt(tx * tx + ty * ty);  // 与目标在x轴（横轴）的距离
  float y = tz;                       // 与目标在y轴（纵轴）的距离

  TestResult tres = test(x, y, M_PI / 4.0);
  if (!tres.ok)
  {
    // __LOG_DEBUG("无法命中");
    return AimResult(false);
  }

  // 二分法
  double lo = -M_PI / 4.0, hi = M_PI / 4.0, mid = 0;
  while (hi - lo > 0.00001)
  {
    // __LOG_DEBUG("{}, {}, {}", lo, hi, mid);
    tres = test(x, y, mid);

    if (std::abs(tres.y_deviation) < 0.00001) break;

    if (tres.y_deviation < 0)
      hi = mid;
    else
      lo = mid;

    mid = (lo + hi) / 2.0;
  }

  return AimResult(true, yaw, mid, tres.time);
}
}  // namespace rm_predict