#ifndef __GEOMETRY_PNP_SOLVER_HPP__
#define __GEOMETRY_PNP_SOLVER_HPP__

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "common/data.hpp"

namespace rm_autoaim
{
class PnpSolver
{
 private:
  cv::Mat camera_matrix_, distortion_coefficients_;

 public:
  PnpSolver() = default;
  PnpSolver(cv::Mat camera_matrix, cv::Mat distortion_coefficients);
  PnpSolver(std::string camera_calibration_file);

  void load(std::string camera_calibration_file);

  Eigen::Vector3d solve(int armor_type, const cv::Point2f pts[]);
};
}  // namespace rm_autoaim

#endif