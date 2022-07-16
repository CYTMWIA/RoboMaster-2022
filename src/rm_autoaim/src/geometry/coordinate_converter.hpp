#ifndef __GEOMETRY_COORDINATE_CONVERTER_HPP__
#define __GEOMETRY_COORDINATE_CONVERTER_HPP__

#include <Eigen/Dense>

namespace rm_autoaim
{

class CoordinateConverter
{
 private:
  Eigen::Vector3d camera_offset_;
  Eigen::Vector3d gimbal_rpy_;

 public:
  void gimbal_angle(const double& pitch, const double& yaw);
  void camera_offset(const double& x, const double& y, const double& z);
  Eigen::Vector3d camera2robot(const Eigen::Vector3d& camera_xyz);
};

}  // namespace rm_autoaim

#endif