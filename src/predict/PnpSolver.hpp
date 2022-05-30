#ifndef __PNP_SOLVER_HPP__
#define __PNP_SOLVER_HPP__

#include "rm_data.hpp"

#include <vector>

#include <opencv2/opencv.hpp>

namespace rmcv::predict
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
}

#endif