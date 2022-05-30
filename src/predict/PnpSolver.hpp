#ifndef __PNP_SOLVER_HPP__
#define __PNP_SOLVER_HPP__

#include <vector>

#include <opencv2/opencv.hpp>

namespace rmcv::predict
{
    enum ArmorType
    {
        kSmallArmor = 0,
        kBigArmor
    };

    // 装甲板尺寸
    const double kSmallArmorWidth = 135;
    const double kSmallArmorHeight = 55;
    const double kBigArmorWidth = 270;
    const double kBigArmorHeight = 55;

    using ObjectPoints = std::vector<cv::Point3f>;
    const std::vector<ObjectPoints> kArmorPoints = {
        {
            cv::Point3f(-kSmallArmorWidth/2, kSmallArmorHeight/2, 0),
            cv::Point3f(-kSmallArmorWidth/2, -kSmallArmorHeight/2, 0),
            cv::Point3f(kSmallArmorWidth/2, -kSmallArmorHeight/2, 0),
            cv::Point3f(kSmallArmorWidth/2, kSmallArmorHeight/2, 0)
        },
        {
            cv::Point3f(-kBigArmorWidth/2, kBigArmorHeight/2, 0),
            cv::Point3f(-kBigArmorWidth/2, -kBigArmorHeight/2, 0),
            cv::Point3f(kBigArmorWidth/2, -kBigArmorHeight/2, 0),
            cv::Point3f(kBigArmorWidth/2, kBigArmorHeight/2, 0)
        }
    };

    struct CameraPose
    {
        double x, y, z;
        double theta_x, theta_y, theta_z;
        double distance();
    };

    class PnpSolver
    {
    private:
        cv::Mat camera_matrix_, distortion_coefficients_;
    public:
        PnpSolver() = delete;

        PnpSolver(cv::Mat camera_matrix, cv::Mat distortion_coefficients);
        PnpSolver(std::string camera_calibration_file);

        CameraPose solve(ArmorType armor_type, const cv::Point2f pts[]);
    };
}

#endif