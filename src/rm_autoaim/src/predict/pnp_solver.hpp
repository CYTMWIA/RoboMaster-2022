#ifndef __PREDICT_PNP_SOLVER_HPP__
#define __PREDICT_PNP_SOLVER_HPP__

#include <opencv2/opencv.hpp>
#include <vector>

#include "rm_common/data.hpp"

namespace rm_autoaim
{
struct TargetPosition
{
  double pitch, yaw, distance;
  double x, y, z;
};

class PnpSolver
{
 private:
  cv::Mat camera_matrix_, distortion_coefficients_;

 public:
  PnpSolver() = delete;

  PnpSolver(cv::Mat camera_matrix, cv::Mat distortion_coefficients);
  PnpSolver(std::string camera_calibration_file);

  TargetPosition solve(rm_data::ArmorType armor_type, const cv::Point2f pts[]);
};
}  // namespace rm_autoaim

#endif