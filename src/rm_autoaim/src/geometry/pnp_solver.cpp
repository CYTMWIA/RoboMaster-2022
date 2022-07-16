#include "pnp_solver.hpp"

#include "common/data.hpp"

namespace rm_autoaim
{
const std::vector<std::vector<cv::Point3f> > ARMOR_POINTS = {
    {cv::Point3f(-rm_data::ARMOR_SMALL_WIDTH / 2, rm_data::LIGHTBAR_HEIGHT / 2, 0),
     cv::Point3f(-rm_data::ARMOR_SMALL_WIDTH / 2, -rm_data::LIGHTBAR_HEIGHT / 2, 0),
     cv::Point3f(rm_data::ARMOR_SMALL_WIDTH / 2, -rm_data::LIGHTBAR_HEIGHT / 2, 0),
     cv::Point3f(rm_data::ARMOR_SMALL_WIDTH / 2, rm_data::LIGHTBAR_HEIGHT / 2, 0)},
    {cv::Point3f(-rm_data::ARMOR_BIG_WIDTH / 2, rm_data::LIGHTBAR_HEIGHT / 2, 0),
     cv::Point3f(-rm_data::ARMOR_BIG_WIDTH / 2, -rm_data::LIGHTBAR_HEIGHT / 2, 0),
     cv::Point3f(rm_data::ARMOR_BIG_WIDTH / 2, -rm_data::LIGHTBAR_HEIGHT / 2, 0),
     cv::Point3f(rm_data::ARMOR_BIG_WIDTH / 2, rm_data::LIGHTBAR_HEIGHT / 2, 0)},
    {cv::Point3f(-rm_data::ARMOR_BIG_WIDTH / 2, rm_data::ARMOR_BIG_HEIGHT / 2, 0),
     cv::Point3f(-rm_data::ARMOR_BIG_WIDTH / 2, -rm_data::ARMOR_BIG_HEIGHT / 2, 0),
     cv::Point3f(rm_data::ARMOR_BIG_WIDTH / 2, -rm_data::ARMOR_BIG_HEIGHT / 2, 0),
     cv::Point3f(rm_data::ARMOR_BIG_WIDTH / 2, rm_data::ARMOR_BIG_HEIGHT / 2, 0)}};

PnpSolver::PnpSolver(cv::Mat camera_matrix, cv::Mat distortion_coefficients)
    : camera_matrix_(camera_matrix), distortion_coefficients_(distortion_coefficients)
{
}

PnpSolver::PnpSolver(std::string camera_calibration_file) { load(camera_calibration_file); }

void PnpSolver::load(std::string camera_calibration_file)
{
  cv::FileStorage fs{camera_calibration_file, cv::FileStorage::READ};
  fs["camera_matrix"] >> camera_matrix_;
  fs["distortion_coefficients"] >> distortion_coefficients_;
}

Eigen::Vector3d PnpSolver::solve(int armor_type, const cv::Point2f pts[])
{
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  std::vector<cv::Point2f> points{pts, pts + 4};

  /* 似乎目标点中x、y、z的顺序并不会影响OpenCV对xyz坐标系的定义
   * 即OpenCV坐标系始终为：y为高度轴，xOz为地面平面（z为俯视状态下的纵轴）
   * （x、y、z分别为tvec第1、2、3个数）
   */
  cv::solvePnP(ARMOR_POINTS[armor_type], points, camera_matrix_, distortion_coefficients_, rvec,
               tvec, false, cv::SOLVEPNP_IPPE);

  Eigen::Vector3d res;

  // 统一坐标系 x y z
  res[0] = tvec.at<double>(0, 0);
  res[1] = tvec.at<double>(2, 0);
  res[2] = -tvec.at<double>(1, 0);

  return res;
}
}  // namespace rm_autoaim