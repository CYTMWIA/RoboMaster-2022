#include "coordinate_converter.hpp"

#include <Eigen/src/Core/Matrix.h>

namespace rm_autoaim
{

void CoordinateConverter::gimbal_angle(const double& pitch, const double& yaw)
{
  gimbal_rpy_[1] = pitch * (M_PI / 180.0);
  gimbal_rpy_[2] = yaw * (M_PI / 180.0);
}

void CoordinateConverter::camera_offset(const double& x, const double& y, const double& z)
{
  camera_offset_[0] = x;
  camera_offset_[1] = y;
  camera_offset_[2] = z;
}

Eigen::Vector3d CoordinateConverter::camera2robot(const Eigen::Vector3d& camera_xyz)
{
  // 转换为 机器人坐标系
  Eigen::Vector3d robot = camera_xyz;
  // 机器人中心
  robot += camera_offset_;
  // Pitch 轴
  double len = sqrt(robot[1] * robot[1] + robot[2] * robot[2]);
  double rad = atan2(robot[2], robot[1]) + gimbal_rpy_[1];
  robot[1] = len * cos(rad);
  robot[2] = len * sin(rad);
  // Yaw 轴
  len = sqrt(robot[0] * robot[0] + robot[1] * robot[1]);
  rad = atan2(robot[1], robot[0]) + gimbal_rpy_[2];
  robot[0] = len * cos(rad);
  robot[1] = len * sin(rad);

  return robot;
}

}  // namespace rm_autoaim