#include "PnpSolver.hpp"

namespace rmcv::predict
{
    PnpSolver::PnpSolver(cv::Mat camera_matrix, cv::Mat distortion_coefficients) : camera_matrix_(camera_matrix),
                                                                                   distortion_coefficients_(distortion_coefficients)
    {
    }

    PnpSolver::PnpSolver(std::string camera_calibration_file)
    {
        cv::FileStorage fs{camera_calibration_file, cv::FileStorage::READ};
        fs["camera_matrix"] >> camera_matrix_;
        fs["distortion_coefficients"] >> distortion_coefficients_;
    }

    TargetPosition PnpSolver::solve(ArmorType armor_type, const cv::Point2f pts[])
    {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        std::vector<cv::Point2f> points{pts, pts + 4};

        /* 似乎目标点中x、y、z的顺序并不会影响OpenCV对xyz坐标系的定义
         * 即OpenCV坐标系始终为：y为高度轴，xOz为地面平面（z为俯视状态下的纵轴）
         * （x、y、z分别为tvec第1、2、3个数）
         */
        cv::solvePnP(kArmorPoints[armor_type], points, camera_matrix_, distortion_coefficients_, rvec, tvec, false, cv::SOLVEPNP_EPNP);

        TargetPosition res;

        // 统一坐标系
        res.x = tvec.at<double>(0, 0);
        res.y = tvec.at<double>(2, 0);
        res.z = -tvec.at<double>(1, 0);

        res.pitch = atan(res.z / sqrt(res.x * res.x + res.y * res.y));
        res.yaw = -atan(res.x / res.y);
        res.distance = sqrt(res.x * res.x + res.y * res.y + res.z * res.z);

        return res;
    }
}