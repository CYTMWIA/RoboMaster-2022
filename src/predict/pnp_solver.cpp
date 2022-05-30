#include "pnp_solver.hpp"

namespace rmcv::predict
{
    double CameraPose::distance()
    {
        return sqrt(x*x+y*y+z*z);
    }

    PnpSolver::PnpSolver(cv::Mat camera_matrix, cv::Mat distortion_coefficients):
        camera_matrix_(camera_matrix),
        distortion_coefficients_(distortion_coefficients)
    { }

    PnpSolver::PnpSolver(std::string camera_calibration_file)
    {
        cv::FileStorage fs{camera_calibration_file, cv::FileStorage::READ};
        fs["camera_matrix"] >> camera_matrix_;
        fs["distortion_coefficients"] >> distortion_coefficients_;
    }

    CameraPose PnpSolver::solve(ArmorType armor_type, const cv::Point2f pts[])
    {
        cv::Mat rvecs = cv::Mat::zeros(3,1,CV_64FC1);
        cv::Mat tvecs = cv::Mat::zeros(3,1,CV_64FC1);
        std::vector<cv::Point2f> points{pts, pts+4};
        
        cv::solvePnP(kArmorPoints[armor_type], points, camera_matrix_, distortion_coefficients_, rvecs, tvecs, false, cv::SOLVEPNP_P3P);

        CameraPose cp;
        cp.x = tvecs.at<double>(0, 0);
        cp.y = tvecs.at<double>(1, 0);
        cp.z = tvecs.at<double>(2, 0);
        cp.rx = rvecs.at<double>(0, 0);
        cp.ry = rvecs.at<double>(1, 0);
        cp.rz = rvecs.at<double>(2, 0);

        return cp;
    }
}